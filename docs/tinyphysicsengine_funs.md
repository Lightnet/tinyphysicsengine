# Notes:
  This use by Grok AI agent to list the functions for reference. 

# tinyphysicsengine (TPE) API Documentation

Below is a comprehensive list of all functions declared in the provided header file (_TINYPHYSICSENGINE_H). The information is extracted directly from the code, including:

- Function Name: The name of the function.
- Return Type: The type returned by the function (e.g., TPE_Vec3, void, uint8_t).
- Parameters: A list of parameters with their types and names (e.g., (TPE_Vec3 point, TPE_Vec3 boxCenter)). If no parameters, noted as "None".
- Description: The doc string or comment associated with the function, if available. If no explicit comment, inferred from context or noted as "N/A (internal or undocumented)".
- Notes: Any additional details, such as if it's static inline, macro-based, or internal.

Functions are grouped by category for readability (e.g., Vector Operations, Body Management), based on their logical grouping in the code. Private/internal functions (e.g., prefixed with _TPE_) are included but marked as such. 
## 1. Vector and Math Operations

|Function Name|Return Type|Parameters|Description|
|---|---|---|---|
|TPE_vec3KeepWithinBox|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 boxCenter, TPE_Vec3 boxMaxVect)|Keeps given point within specified axis-aligned box. This can be used e.g. to smooth rendered movement of jittering physics bodies.|
|TPE_vec3KeepWithinDistanceBand|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Unit minDistance, TPE_Unit maxDistance)|Keeps the point within a distance band from the center (between min and max distance).|
|TPE_vec3|TPE_Vec3|(TPE_Unit x, TPE_Unit y, TPE_Unit z)|Creates a new TPE_Vec3 vector from x, y, z components.|
|TPE_vec3Minus|TPE_Vec3|(TPE_Vec3 v1, TPE_Vec3 v2)|Subtracts v2 from v1.|
|TPE_vec3Plus|TPE_Vec3|(TPE_Vec3 v1, TPE_Vec3 v2)|Adds v1 and v2.|
|TPE_vec3Cross|TPE_Vec3|(TPE_Vec3 v1, TPE_Vec3 v2)|Computes the cross product of v1 and v2.|
|TPE_vec3Project|TPE_Vec3|(TPE_Vec3 v, TPE_Vec3 base)|Projects vector v onto base (base does not need to be normalized).|
|TPE_vec3ProjectNormalized|TPE_Vec3|(TPE_Vec3 v, TPE_Vec3 baseNormalized)|Projects vector v onto a normalized base vector.|
|TPE_vec3Times|TPE_Vec3|(TPE_Vec3 v, TPE_Unit units)|Multiplies vector v by a fixed-point scalar (units / TPE_F).|
|TPE_vec3TimesPlain|TPE_Vec3|(TPE_Vec3 v, TPE_Unit q)|Multiplies vector v by a plain integer scalar q (no fixed-point adjustment).|
|TPE_vec3Normalized|TPE_Vec3|(TPE_Vec3 v)|Returns a normalized copy of v.|
|TPE_vec3Dot|TPE_Unit|(TPE_Vec3 v1, TPE_Vec3 v2)|Computes the dot product of v1 and v2.|
|TPE_vec3Len|TPE_Unit|(TPE_Vec3 v)|Computes the exact length of vector v using sqrt.|
|TPE_vec3LenApprox|TPE_Unit|(TPE_Vec3 v)|Computes an approximate length of vector v (faster but less accurate).|
|TPE_vec2Angle|TPE_Unit|(TPE_Unit x, TPE_Unit y)|Returns an angle in TPE_Units of a 2D vector with the X axis, CCW.|
|TPE_keepInRange|TPE_Unit|(TPE_Unit x, TPE_Unit xMin, TPE_Unit xMax)|Keeps given value within specified range. This can be used e.g. for movement smoothing.|
|TPE_abs|TPE_Unit|(TPE_Unit x)|Returns the absolute value of x. (Static inline)|
|TPE_max|TPE_Unit|(TPE_Unit a, TPE_Unit b)|Returns the maximum of a and b. (Static inline)|
|TPE_min|TPE_Unit|(TPE_Unit a, TPE_Unit b)|Returns the minimum of a and b. (Static inline)|
|TPE_nonZero|TPE_Unit|(TPE_Unit x)|Returns x if non-zero, otherwise 1 to prevent division by zero. (Static inline)|
|TPE_dist|TPE_Unit|(TPE_Vec3 p1, TPE_Vec3 p2)|Computes the exact distance between p1 and p2. (Static inline)|
|TPE_distApprox|TPE_Unit|(TPE_Vec3 p1, TPE_Vec3 p2)|Computes an approximate distance between p1 and p2. (Static inline)|
|TPE_sqrt|TPE_Unit|(TPE_Unit x)|Computes the square root of x (handles negative values by sign).|
|TPE_sin|TPE_Unit|(TPE_Unit x)|Computes sine, where TPE_FRACTIONS_PER_UNIT corresponds to 2π radians. Returns value from -TPE_F to TPE_F.|
|TPE_cos|TPE_Unit|(TPE_Unit x)|Computes cosine (implemented as sin(x + π/2)).|
|TPE_atan|TPE_Unit|(TPE_Unit x)|Computes arctangent approximation (polynomial-based).|
|TPE_vec3Normalize|void|(TPE_Vec3 *v)|Normalizes the vector v in place.|
|TPE_pointRotate|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 rotation)|Rotates a 3D point by given Euler angle rotation (ZXY order).|
|TPE_rotationInverse|TPE_Vec3|(TPE_Vec3 rotation)|Returns an inverse rotation to given rotation, in Euler angles.|
|TPE_rotationRotateByAxis|TPE_Vec3|(TPE_Vec3 rotation, TPE_Vec3 rotationByAxis)|Rotates a rotation specified in Euler angles by given axis + angle.|
|TPE_rotationFromVecs|TPE_Vec3|(TPE_Vec3 forward, TPE_Vec3 right)|Computes orientation/rotation from two vectors (should be perpendicular).|

## 2. Joint and Connection Operations

|Function Name|Return Type|Parameters|Description|
|---|---|---|---|
|TPE_joint|TPE_Joint|(TPE_Vec3 position, TPE_Unit size)|Creates a new TPE_Joint with given position and size.|
|TPE_connectionTension|TPE_Unit|(TPE_Unit length, TPE_Unit desiredLength)|Returns a connection tension, i.e., a signed percentage difference against desired length (TPE_F means 100%). (Static inline)|
|TPE_jointPin|void|(TPE_Joint *joint, TPE_Vec3 position)|Pins a joint to a specified location (sets position and zeros velocity).|
|TPE_jointHash|uint32_t|(const TPE_Joint *joint)|Computes a 32-bit hash of the joint.|
|TPE_connectionHash|uint32_t|(const TPE_Connection *connection)|Computes a 32-bit hash of the connection.|

## 3. Body Management and Simulation

|Function Name|Return Type|Parameters|Description|
|---|---|---|---|
|TPE_bodyInit|void|(TPE_Body *body, TPE_Joint *joints, uint8_t jointCount, TPE_Connection *connections, uint8_t connectionCount, TPE_Unit mass)|Initializes a TPE_Body with joints, connections, and mass.|
|TPE_bodyGetRotation|TPE_Vec3|(const TPE_Body *body, uint16_t joint1, uint16_t joint2, uint16_t joint3)|Gets orientation (rotation) of a body from positions of three joints (ZXY Euler angles).|
|TPE_bodyGetCenterOfMass|TPE_Vec3|(const TPE_Body *body)|Computes the center of mass of a body by averaging joint positions.|
|TPE_bodyDeactivate|void|(TPE_Body *body)|Deactivates a body (puts it to sleep until collision or force).|
|TPE_bodyActivate|void|(TPE_Body *body)|Activates a body (wakes it from deactivation).|
|TPE_bodyGetNetSpeed|TPE_Unit|(const TPE_Body *body)|Computes the net speed of the body (sum of joint speeds, approximate if defined).|
|TPE_bodyGetAverageSpeed|TPE_Unit|(const TPE_Body *body)|Computes the average speed of the body (net speed / joint count).|
|TPE_bodyMultiplyNetSpeed|void|(TPE_Body *body, TPE_Unit factor)|Multiplies the net speed of all joints by a factor.|
|TPE_bodyLimitAverageSpeed|void|(TPE_Body *body, TPE_Unit speedMin, TPE_Unit speedMax)|Limits the average speed of the body to a range.|
|TPE_bodyReshape|void|(TPE_Body *body, TPE_ClosestPointFunction environmentFunction)|Attempts to shift joints of a soft body to zero spring tension.|
|TPE_bodyCancelOutVelocities|void|(TPE_Body *body, uint8_t strong)|Performs "magic" on body connections (cancels antagonist velocities, applies friction). Mostly internal.|
|TPE_bodyMoveBy|void|(TPE_Body *body, TPE_Vec3 offset)|Moves a body by a certain offset.|
|TPE_bodyMoveTo|void|(TPE_Body *body, TPE_Vec3 position)|Moves a body's center of mass to a given position.|
|TPE_bodyStop|void|(TPE_Body *body)|Zeros velocities of all joints.|
|TPE_bodyAccelerate|void|(TPE_Body *body, TPE_Vec3 velocity)|Adds velocity to all joints.|
|TPE_bodyApplyGravity|void|(TPE_Body *body, TPE_Unit downwardsAccel)|Applies gravity acceleration to all joints.|
|TPE_bodySpin|void|(TPE_Body *body, TPE_Vec3 rotation)|Adds angular velocity to the body (axis + magnitude).|
|TPE_bodySpinWithCenter|void|(TPE_Body *body, TPE_Vec3 rotation, TPE_Vec3 center)|Adds angular velocity with a specified center.|
|TPE_bodyRotateByAxis|void|(TPE_Body *body, TPE_Vec3 rotation)|Instantly rotates a body about an axis.|
|TPE_bodyGetAABB|void|(const TPE_Body *body, TPE_Vec3 *vMin, TPE_Vec3 *vMax)|Computes the minimum axis-aligned bounding box of the body.|
|TPE_bodyGetFastBSphere|void|(const TPE_Body *body, TPE_Vec3 *center, TPE_Unit *radius)|Computes a fast (not minimal) bounding sphere.|
|TPE_bodyGetBSphere|void|(const TPE_Body *body, TPE_Vec3 *center, TPE_Unit *radius)|Computes the minimum bounding sphere.|
|TPE_bodyGetLinearVelocity|TPE_Vec3|(const TPE_Body *body)|Computes the average linear velocity of the body.|
|TPE_bodyHash|uint32_t|(const TPE_Body *body)|Computes a 32-bit hash of the body.|
|TPE_bodyIsActive|uint8_t|(const TPE_Body *body)|Returns 1 if the body is active (not deactivated). (Static inline)|

## 4. World and Collision Operations

|Function Name|Return Type|Parameters|Description|
|---|---|---|---|
|TPE_worldInit|void|(TPE_World *world, TPE_Body *bodies, uint16_t bodyCount, TPE_ClosestPointFunction environmentFunction)|Initializes a TPE_World with bodies and environment function.|
|TPE_worldStep|void|(TPE_World *world)|Performs one simulation step (tick) of the physics world.|
|TPE_worldDeactivateAll|void|(TPE_World *world)|Deactivates all bodies in the world.|
|TPE_worldActivateAll|void|(TPE_World *world)|Activates all bodies in the world.|
|TPE_worldGetNetSpeed|TPE_Unit|(const TPE_World *world)|Computes the net speed of all bodies in the world.|
|TPE_worldHash|uint32_t|(const TPE_World *world)|Computes a 32-bit hash of the world state.|
|TPE_getVelocitiesAfterCollision|void|(TPE_Unit *v1, TPE_Unit *v2, TPE_Unit m1, TPE_Unit m2, TPE_Unit elasticity)|Computes velocities after a 1D collision of rigid bodies.|
|TPE_jointsResolveCollision|uint8_t|(TPE_Joint *j1, TPE_Joint *j2, TPE_Unit mass1, TPE_Unit mass2, TPE_Unit elasticity, TPE_Unit friction, TPE_ClosestPointFunction env)|Resolves collision between two joints, keeping them outside environment. Returns 1 if collided. Mostly internal.|
|TPE_jointEnvironmentResolveCollision|uint8_t|(TPE_Joint *joint, TPE_Unit elasticity, TPE_Unit friction, TPE_ClosestPointFunction env)|Resolves joint-environment collision. Returns 0 (no collision), 1 (resolved), or 2 (failed). Mostly internal.|
|TPE_bodyEnvironmentCollide|uint8_t|(const TPE_Body *body, TPE_ClosestPointFunction env)|Tests if a body is colliding with the environment.|
|TPE_bodyEnvironmentResolveCollision|uint8_t|(TPE_Body *body, TPE_ClosestPointFunction env)|Resolves body-environment collision. Returns 1 if collision happened. Mostly internal.|
|TPE_checkOverlapAABB|uint8_t|(TPE_Vec3 v1Min, TPE_Vec3 v1Max, TPE_Vec3 v2Min, TPE_Vec3 v2Max)|Checks if two axis-aligned bounding boxes overlap.|
|TPE_bodiesResolveCollision|uint8_t|(TPE_Body *b1, TPE_Body *b2, TPE_ClosestPointFunction env)|Resolves collision between two bodies. Returns 1 if collision happened. Mostly internal.|
|TPE_worldDebugDraw|void|(TPE_World *world, TPE_DebugDrawFunction drawFunc, TPE_Vec3 camPos, TPE_Vec3 camRot, TPE_Vec3 camView, uint16_t envGridRes, TPE_Unit envGridSize)|Draws a debug view of the 3D physics world using a pixel drawing function.|
|TPE_castEnvironmentRay|TPE_Vec3|(TPE_Vec3 rayPos, TPE_Vec3 rayDir, TPE_ClosestPointFunction environment, TPE_Unit insideStepSize, TPE_Unit rayMarchMaxStep, uint32_t maxSteps)|Casts a ray against the environment and returns the closest hit point.|
|TPE_castBodyRay|TPE_Vec3|(TPE_Vec3 rayPos, TPE_Vec3 rayDir, int16_t excludeBody, const TPE_World *world, int16_t *bodyIndex, int16_t *jointIndex)|Casts a ray against bodies in the world, returns closest hit position and indices.|
|TPE_testClosestPointFunction|uint8_t|(TPE_ClosestPointFunction f, TPE_Vec3 cornerFrom, TPE_Vec3 cornerTo, uint8_t gridResolution, TPE_UnitReduced allowedError, TPE_Vec3 *errorPoint)|Tests the validity of a closest point function (SDF-like). Returns 1 if valid.|

## 5. Body Generation Helpers

|Function Name|Return Type|Parameters|Description|
|---|---|---|---|
|TPE_makeBox|void|(TPE_Joint joints[8], TPE_Connection connections[16], TPE_Unit width, TPE_Unit depth, TPE_Unit height, TPE_Unit jointSize)|Generates joints and connections for a box shape.|
|TPE_makeCenterBox|void|(TPE_Joint joints[9], TPE_Connection connections[18], TPE_Unit width, TPE_Unit depth, TPE_Unit height, TPE_Unit jointSize)|Generates a box with a center joint.|
|TPE_makeRect|void|(TPE_Joint joints[4], TPE_Connection connections[6], TPE_Unit width, TPE_Unit depth, TPE_Unit jointSize)|Generates a rectangle shape.|
|TPE_makeTriangle|void|(TPE_Joint joints[3], TPE_Connection connections[3], TPE_Unit sideLength, TPE_Unit jointSize)|Generates a triangle shape.|
|TPE_makeCenterRect|void|(TPE_Joint joints[5], TPE_Connection connections[8], TPE_Unit width, TPE_Unit depth, TPE_Unit jointSize)|Generates a rectangle with a center joint.|
|TPE_makeCenterRectFull|void|(TPE_Joint joints[5], TPE_Connection connections[10], TPE_Unit width, TPE_Unit depth, TPE_Unit jointSize)|Generates a full rectangle with center and extra connections.|
|TPE_make2Line|void|(TPE_Joint joints[2], TPE_Connection connections[1], TPE_Unit length, TPE_Unit jointSize)|Generates a simple 2-point line.|

## 6. Environment Shape Functions

|Function Name|Return Type|Parameters|Description|
|---|---|---|---|
|TPE_envAABoxInside|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 size)|Closest point function for inside an axis-aligned box.|
|TPE_envAABox|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 maxCornerVec)|Closest point function for an axis-aligned box.|
|TPE_envBox|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 maxCornerVec, TPE_Vec3 rotation)|Closest point function for a rotated box.|
|TPE_envSphere|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Unit radius)|Closest point function for a sphere.|
|TPE_envSphereInside|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Unit radius)|Closest point function for inside a sphere.|
|TPE_envHalfPlane|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 normal)|Closest point function for a half-plane.|
|TPE_envGround|TPE_Vec3|(TPE_Vec3 point, TPE_Unit height)|Closest point function for a ground plane at given height.|
|TPE_envInfiniteCylinder|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 direction, TPE_Unit radius)|Closest point function for an infinite cylinder.|
|TPE_envCylinder|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 direction, TPE_Unit radius)|Closest point function for a finite cylinder.|
|TPE_envCone|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Vec3 direction, TPE_Unit radius)|Closest point function for a cone.|
|TPE_envLineSegment|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 a, TPE_Vec3 b)|Closest point function for a line segment.|
|TPE_envHeightmap|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, TPE_Unit gridSize, TPE_Unit (*heightFunction)(int32_t x, int32_t y), TPE_Unit maxDist)|Closest point function for a heightmap defined by a function.|
|TPE_envAATriPrism|TPE_Vec3|(TPE_Vec3 point, TPE_Vec3 center, const TPE_Unit sides[6], TPE_Unit depth, uint8_t direction)|Closest point function for a triangular prism (e.g., ramps). Points must be CCW.|

## 7. Internal/Private Functions (Prefixed with _TPE_)

These are not intended for direct use but are part of the implementation.

|Function Name|Return Type|Parameters|Description|
|---|---|---|---|
|_TPE_rotateByAxis|TPE_Vec3|(TPE_Vec3 p, TPE_Vec3 axisNormalized, TPE_Unit angle)|Internal rotation by axis (used in rotations). N/A (internal).|
|_TPE_vec2Rotate|void|(TPE_Unit *x, TPE_Unit *y, TPE_Unit angle)|Internal 2D vector rotation. N/A (internal).|
|_TPE_project3DPoint|TPE_Vec3|(TPE_Vec3 p, TPE_Vec3 camPos, TPE_Vec3 camRot, TPE_Vec3 camView)|Internal 3D point projection for debug drawing. N/A (internal).|
|_TPE_drawDebugPixel|void|(TPE_Unit x, TPE_Unit y, TPE_Unit w, TPE_Unit h, uint8_t c, TPE_DebugDrawFunction f)|Internal pixel drawing for debug. N/A (internal).|
|_TPE_hash|uint32_t|(uint32_t n)|Internal hash function. N/A (internal).|
|_TPE_bodyNonrotatingJointCollided|void|(TPE_Body *b, int16_t jointIndex, TPE_Vec3 origPos, uint8_t success)|Internal handling for non-rotating body collisions. N/A (internal).|

This covers all functions in the header. Macros (e.g., TPE_ENV_START, TPE_PRINTF_VEC3) are not included as they are not functions.