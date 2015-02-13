/*global define*/
define([
        '../Core/Cartesian3',
        '../Core/ComponentDatatype',
        '../Core/defaultValue',
        '../Core/defined',
        '../Core/Ellipsoid',
        '../Core/EllipsoidTangentPlane',
        '../Core/EncodedCartesian3',
        '../Core/IndexDatatype',
        '../Core/Math',
        '../Core/Matrix4',
        '../Core/PolygonGeometryLibrary',
        '../Core/PolygonPipeline',
        '../Core/PrimitiveType',
        '../Core/WindingOrder',
        '../Renderer/BufferUsage',
        '../Renderer/DrawCommand',
        '../Shaders/ShadowVolumeFS',
        '../Shaders/ShadowVolumeVS',
        './BlendingState',
        './CullFace',
        './DepthFunction',
        './Pass',
        './StencilFunction',
        './StencilOperation'
    ], function(
        Cartesian3,
        ComponentDatatype,
        defaultValue,
        defined,
        Ellipsoid,
        EllipsoidTangentPlane,
        EncodedCartesian3,
        IndexDatatype,
        CesiumMath,
        Matrix4,
        PolygonGeometryLibrary,
        PolygonPipeline,
        PrimitiveType,
        WindingOrder,
        BufferUsage,
        DrawCommand,
        ShadowVolumeFS,
        ShadowVolumeVS,
        BlendingState,
        CullFace,
        DepthFunction,
        Pass,
        StencilFunction,
        StencilOperation) {
    "use strict";

    var PolygonOnTerrain = function(options) {
        options = defaultValue(options, defaultValue.EMPTY_OBJECT);

        var ellipsoid = defaultValue(options.ellipsoid, Ellipsoid.WGS84);
        var granularity = defaultValue(options.granularity, CesiumMath.RADIANS_PER_DEGREE);
        var polygonHierarchy = options.polygonHierarchy;

        this._ellipsoid = ellipsoid;
        this._granularity = granularity;
        this._polygonHierarchy = polygonHierarchy;

        this._va = undefined;
        this._sp = undefined;
        this._rs = undefined;

        this._zFailCommand = undefined;
        this._zPassCommand = undefined;
        this._colorInsideSphereCommand = undefined;
        this._colorOutsideSphereCommand = undefined;
    };

    var attributeLocations = {
        positionHigh : 0,
        positionLow : 1,
        normal : 2
    };

    function getSurfaceDelta(ellipsoid, granularity) {
        var refDistance = ellipsoid.maximumRadius;
        return refDistance - (refDistance * Math.cos(granularity / 2.0));
    }

    var scratchPosition = new Cartesian3();
    var scratchNormal = new Cartesian3();
    var scratchDeltaNormal = new Cartesian3();

    // TODO: More than one level of the polygon hierarchy
    function createShadowVolume(polygon, context) {
        var polygonHierarchy = polygon._polygonHierarchy;
        var ellipsoid = polygon._ellipsoid;
        var granularity = polygon._granularity;

        var results = PolygonGeometryLibrary.polygonsFromHierarchy(polygonHierarchy);
        var positions = results.polygons[0];
        var hierarchy = results.hierarchy[0];

        var bottomCap = PolygonGeometryLibrary.createGeometryFromPositions(ellipsoid, positions, granularity, false);
        var bottomPositions = bottomCap.attributes.position.values;
        var numBottomCapVertices = bottomPositions.length / 3;
        var numCapVertices = numBottomCapVertices + numBottomCapVertices;
        var bottomIndices = bottomCap.indices;
        var numBottomIndices = bottomIndices.length;
        var numCapIndices = numBottomIndices + numBottomIndices;

        var outerRing = hierarchy.outerRing;
        var tangentPlane = EllipsoidTangentPlane.fromPoints(outerRing, ellipsoid);
        var positions2D = tangentPlane.projectPointsOntoPlane(outerRing);

        var windingOrder = PolygonPipeline.computeWindingOrder2D(positions2D);
        if (windingOrder === WindingOrder.CLOCKWISE) {
            outerRing.reverse();
        }

        var wall = PolygonGeometryLibrary.computeWallGeometry(outerRing, ellipsoid, granularity, false);
        var walls = [wall];
        var numWallVertices = wall.attributes.position.values.length / 3;
        var numWallIndices = wall.indices.length;

        var holes = hierarchy.holes;
        var i;
        var j;

        for (i = 0; i < holes.length; i++) {
            var hole = holes[i];

            tangentPlane = EllipsoidTangentPlane.fromPoints(hole, ellipsoid);
            positions2D = tangentPlane.projectPointsOntoPlane(hole);

            windingOrder = PolygonPipeline.computeWindingOrder2D(positions2D);
            if (windingOrder === WindingOrder.COUNTER_CLOCKWISE) {
                hole.reverse();
            }

            walls.push(PolygonGeometryLibrary.computeWallGeometry(hole, ellipsoid, granularity, false));
            numWallVertices += wall.attributes.position.values.length / 3;
            numWallIndices += wall.indices.length;
            walls.push(wall);
        }

        var maxAlt = 8500.0; // TODO: get max alt of terrain
        var surfaceDelta = getSurfaceDelta(ellipsoid, granularity);
        var upDelta = maxAlt + surfaceDelta;

        var numVertices = numCapVertices + numWallVertices;

        var vbPositions = new Float32Array(numVertices * 3 * 2);
        var vbNormals = new Float32Array(numVertices * 3);

        var position;
        var normal;
        var topPosition;

        var index = 0;
        var normalIndex = 0;

        for (i = 0; i < numBottomCapVertices * 3; i += 3) {
            position = Cartesian3.unpack(bottomPositions, i, scratchPosition);
            ellipsoid.scaleToGeodeticSurface(position, position);
            normal = ellipsoid.geodeticSurfaceNormal(position, scratchNormal);

            topPosition = Cartesian3.multiplyByScalar(normal, upDelta, scratchDeltaNormal);
            Cartesian3.add(position, topPosition, topPosition);

            EncodedCartesian3.writeElements(position, vbPositions, index);
            EncodedCartesian3.writeElements(topPosition, vbPositions, index + 6);
            index += 12;

            Cartesian3.pack(normal, vbNormals, normalIndex);
            Cartesian3.pack(Cartesian3.ZERO, vbNormals, normalIndex + 3);
            normalIndex += 6;
        }

        var numWalls = walls.length;
        var wallPositions;
        var wallLength;

        for (i = 0; i < numWalls; ++i) {
            wall = walls[i];
            wallPositions = wall.attributes.position.values;
            wallLength = wallPositions.length / 2;

            for (j = 0; j < wallLength; j += 3) {
                position = Cartesian3.unpack(wallPositions, j, scratchPosition);
                ellipsoid.scaleToGeodeticSurface(position, position);
                normal = ellipsoid.geodeticSurfaceNormal(position, scratchNormal);

                topPosition = Cartesian3.multiplyByScalar(normal, upDelta, scratchDeltaNormal);
                Cartesian3.add(position, topPosition, topPosition);

                EncodedCartesian3.writeElements(topPosition, vbPositions, index);
                EncodedCartesian3.writeElements(position, vbPositions, index + wallLength * 2);
                index += 6;

                Cartesian3.pack(Cartesian3.ZERO, vbNormals, normalIndex);
                Cartesian3.pack(normal, vbNormals, normalIndex + wallLength);
                normalIndex += 3;
            }

            index += wallLength * 2;
            normalIndex += wallLength;
        }

        var numIndices = numCapIndices + numWallIndices;
        var ibIndices = IndexDatatype.createTypedArray(numVertices, numIndices);

        var i0;
        var i1;
        var i2;

        index = 0;
        for (i = 0; i < numBottomIndices; i += 3) {
            i0 = bottomIndices[i] * 2;
            i1 = bottomIndices[i + 1] * 2;
            i2 = bottomIndices[i + 2] * 2;

            ibIndices[index++] = i2;
            ibIndices[index++] = i1;
            ibIndices[index++] = i0;
        }

        for (i = 0; i < numBottomIndices; i += 3) {
            i0 = bottomIndices[i] * 2;
            i1 = bottomIndices[i + 1] * 2;
            i2 = bottomIndices[i + 2] * 2;

            ibIndices[index++] = i0 + 1;
            ibIndices[index++] = i1 + 1;
            ibIndices[index++] = i2 + 1;
        }

        var offset = numCapVertices;
        for (i = 0; i < numWalls; ++i) {
            wall = walls[i];
            var wallIndices = wall.indices;
            wallLength = wallIndices.length;

            for (j = 0; j < wallLength; ++j) {
                ibIndices[index++] = wallIndices[j] + offset;
            }
            offset += wall.attributes.position.values.length / 3;
        }

        var positionBuffer = context.createVertexBuffer(vbPositions, BufferUsage.STATIC_DRAW);
        var normalBuffer = context.createVertexBuffer(vbNormals, BufferUsage.STATIC_DRAW);

        var indexDatatype = (ibIndices.BYTES_PER_ELEMENT === 2) ?  IndexDatatype.UNSIGNED_SHORT : IndexDatatype.UNSIGNED_INT;
        var indexBuffer = context.createIndexBuffer(ibIndices, BufferUsage.STATIC_DRAW, indexDatatype);

        var attributes = [{
            index                  : attributeLocations.positionHigh,
            vertexBuffer           : positionBuffer,
            componentsPerAttribute : 3,
            componentDatatype      : ComponentDatatype.FLOAT,
            offsetInBytes          : 0,
            strideInBytes          : ComponentDatatype.getSizeInBytes(ComponentDatatype.FLOAT) * 3 * 2
        }, {
            index                  : attributeLocations.positionLow,
            vertexBuffer           : positionBuffer,
            componentsPerAttribute : 3,
            componentDatatype      : ComponentDatatype.FLOAT,
            offsetInBytes          : ComponentDatatype.getSizeInBytes(ComponentDatatype.FLOAT) * 3,
            strideInBytes          : ComponentDatatype.getSizeInBytes(ComponentDatatype.FLOAT) * 3 * 2
        }, {
            index                  : attributeLocations.normal,
            vertexBuffer           : normalBuffer,
            componentsPerAttribute : 3,
            componentDatatype      : ComponentDatatype.FLOAT
        }];

        polygon._va = context.createVertexArray(attributes, indexBuffer);

        polygon._bottomCapOffset = 0;
        polygon._bottomCapCount = numBottomIndices;
        polygon._topCapOffset = numBottomIndices;
        polygon._topCapCount = numBottomIndices;
        polygon._wallOffset = numCapIndices;
        polygon._wallCount = numWallIndices;
    }

    PolygonOnTerrain.prototype.update = function(context, frameState, commandList) {

        if (!defined(this._va)) {
            createShadowVolume(this, context);
        }

        if (!defined(this._sp)) {
            this._sp = context.createShaderProgram(ShadowVolumeVS, ShadowVolumeFS, attributeLocations);
        }

        if (!defined(this._zFailCommand)) {
            var uniformMap = {
                centralBodyMinimumAltitude : function() {
                    return -8500.0;
                },
                LODNegativeToleranceOverDistance : function() {
                    return -0.01;
                }
            };

            var disableColorWrites = {
                red : false,
                green : false,
                blue : false,
                alpha : false
            };

            var zFailRenderState = context.createRenderState({
                colorMask : disableColorWrites,
                stencilTest : {
                    //enabled : true,
                    frontFunction : StencilFunction.ALWAYS,
                    frontOperation : {
                        fail : StencilOperation.KEEP,
                        zFail : StencilOperation.DECREMENT_WRAP,
                        zPass : StencilOperation.KEEP
                    },
                    backFunction : StencilFunction.ALWAYS,
                    backOperation : {
                        fail : StencilOperation.KEEP,
                        zFail : StencilOperation.INCREMENT_WRAP,
                        zPass : StencilOperation.KEEP
                    },
                    reference : 0,
                    mask : ~0
                },
                depthTest : {
                    //enabled : true
                },
                depthMask : false
            });

            this._zFailCommand = new DrawCommand({
                primitiveType : PrimitiveType.TRIANGLES,
                vertexArray : this._va,
                renderState : zFailRenderState,
                shaderProgram : this._sp,
                uniformMap : uniformMap,
                owner : this,
                modelMatrix : Matrix4.IDENTITY,
                pass : Pass.TRANSLUCENT
            });

            var zPassRenderState = context.createRenderState({
                colorMask : disableColorWrites,
                stencilTest : {
                    //enabled : true,
                    frontFunction : StencilFunction.ALWAYS,
                    frontOperation : {
                        fail : StencilOperation.KEEP,
                        zFail : StencilOperation.KEEP,
                        zPass : StencilOperation.INCREMENT_WRAP
                    },
                    backFunction : StencilFunction.ALWAYS,
                    backOperation : {
                        fail : StencilOperation.KEEP,
                        zFail : StencilOperation.KEEP,
                        zPass : StencilOperation.DECREMENT_WRAP
                    },
                    reference : 0,
                    mask : ~0
                },
                depthTest : {
                    //enabled : true
                },
                depthMask : false
            });

            this._zPassCommand = new DrawCommand({
                primitiveType : PrimitiveType.TRIANGLES,
                offset : this._topCapOffset,
                count : this._topCapCount + this._wallCount,
                vertexArray : this._va,
                renderState : zPassRenderState,
                shaderProgram : this._sp,
                uniformMap : uniformMap,
                owner : this,
                modelMatrix : Matrix4.IDENTITY,
                pass : Pass.TRANSLUCENT
            });

            var colorStencilTest = {
                //enabled : true,
                frontFunction : StencilFunction.NOT_EQUAL,
                frontOperation : {
                    fail : StencilOperation.KEEP,
                    zFail : StencilOperation.KEEP,
                    zPass : StencilOperation.DECREMENT
                },
                backFunction : StencilFunction.NOT_EQUAL,
                backOperation : {
                    fail : StencilOperation.KEEP,
                    zFail : StencilOperation.KEEP,
                    zPass : StencilOperation.DECREMENT
                },
                reference : 0,
                mask : ~0
            };

            var colorInsideSphereRenderState = context.createRenderState({
                stencilTest : colorStencilTest,
                depthTest : {
                    //enabled : true,
                    func : DepthFunction.ALWAYS
                },
                depthMask : false
            });

            this._colorInsideCommand = new DrawCommand({
                primitiveType : PrimitiveType.TRIANGLES,
                vertexArray : this._va,
                renderState : colorInsideSphereRenderState,
                shaderProgram : this._sp,
                uniformMap : uniformMap,
                owner : this,
                modelMatrix : Matrix4.IDENTITY,
                pass : Pass.TRANSLUCENT
            });

            var colorOutsideSphereRenderState = context.createRenderState({
                stencilTest : colorStencilTest,
                cull : {
                    //enabled : true,
                    face : CullFace.BACK
                },
                depthTest : {
                    //enabled : true
                },
                depthMask : false
            });

            this._colorOutsideSphereCommand = new DrawCommand({
                primitiveType : PrimitiveType.TRIANGLES,
                offset : this._topCapOffset,
                count : this._topCapCount + this._wallCount,
                vertexArray : this._va,
                renderState : colorOutsideSphereRenderState,
                shaderProgram : this._sp,
                uniformMap : uniformMap,
                owner : this,
                modelMatrix : Matrix4.IDENTITY,
                pass : Pass.TRANSLUCENT
            });
        }

        var pass = frameState.passes;
        if (pass.render) {
            // intersects near/far plane: z-fail else z-pass
            // inside bounding sphere : colorInsideSphere commands else color outside
            commandList.push(this._zPassCommand, this._colorOutsideSphereCommand);
        }
    };

    return PolygonOnTerrain;
});
