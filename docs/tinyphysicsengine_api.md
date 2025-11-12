# Overview TinyPhysicsEngine API:

This is base on Grok AI Agent for getting information for functions and parameters.

Here is a complete list of all public functions, types, constants, and parameters in the tinyphysicsengine (TPE) physics API, as defined in the provided header file tinyphysicsengine.h.

---

## 1. Basic Types

| Type            | Description                                    |
| --------------- | ---------------------------------------------- |
| TPE_Unit        | int32_t – Fixed-point unit (512 = 1 full unit) |
| TPE_UnitReduced | int16_t – Space-optimized version of TPE_Unit  |
| TPE_Vec3        | { TPE_Unit x, y, z } – 3D vector               |
| TPE_Joint       | Sphere in soft body: position, velocity, size  |
| TPE_Connection  | Elastic spring between two joints              |
| TPE_Body        | Soft body made of joints + connections         |
| TPE_World       | Collection of bodies + environment             |

---

## 2. Core Constants

| Constant                  | Value      | Meaning                      |
| ------------------------- | ---------- | ---------------------------- |
| TPE_FRACTIONS_PER_UNIT    | 512        | One full unit in fixed-point |
| TPE_F                     | 512        | Shortcut                     |
| TPE_INFINITY              | 2147483647 | Max safe TPE_Unit            |
| TPE_JOINT_SIZE_MULTIPLIER | 32         | Joint size scaling           |

---

## 3. Configuration Macros (User-Overridable)

c
```c
TPE_APPROXIMATE_LENGTH           // 1 = faster approx distance
TPE_LOW_SPEED                    // Speed threshold for deactivation
TPE_RESHAPE_TENSION_LIMIT        // When to reshape stiff bodies
TPE_RESHAPE_ITERATIONS           // Reshape iterations
TPE_DEACTIVATE_AFTER             // Ticks before sleep (≤255)
TPE_LIGHT_DEACTIVATION           // Post-collision deactivation delay
TPE_TENSION_ACCELERATION_DIVIDER // Must be power of 2
TPE_TENSION_ACCELERATION_THRESHOLD
TPE_TENSION_GREATER_ACCELERATION_THRESHOLD
TPE_COLLISION_RESOLUTION_ITERATIONS
TPE_COLLISION_RESOLUTION_MARGIN
TPE_NONROTATING_COLLISION_RESOLVE_ATTEMPTS
TPE_APPROXIMATE_NET_SPEED
```

---

## 4. Body Flags

c
```c
TPE_BODY_FLAG_DEACTIVATED     // Sleeping
TPE_BODY_FLAG_NONROTATING     // No rotation, only translation
TPE_BODY_FLAG_DISABLED        // Ignored in simulation
TPE_BODY_FLAG_SOFT            // Allow deformation
TPE_BODY_FLAG_SIMPLE_CONN     // Skip advanced connection logic
TPE_BODY_FLAG_ALWAYS_ACTIVE   // Never sleep
```

---

## 5. Function Pointer Types

| Type                     | Signature                                                                     |
| ------------------------ | ----------------------------------------------------------------------------- |
| TPE_ClosestPointFunction | TPE_Vec3 (*)(TPE_Vec3 point, TPE_Unit maxDist)                                |
| TPE_CollisionCallback    | uint8_t (*)(uint16_t b1, uint16_t j1, uint16_t b2, uint16_t j2, TPE_Vec3 pos) |
| TPE_DebugDrawFunction    | void (*)(uint16_t x, uint16_t y, uint8_t color)                               |

---

## 6. Vector & Math Functions

| Function                                         | Description                      | Parameters     |
| ------------------------------------------------ | -------------------------------- | -------------- |
| TPE_vec3(x, y, z)                                | Create vector                    | TPE_Unit x,y,z |
| TPE_vec3Plus(v1, v2)                             | v1 + v2                          |                |
| TPE_vec3Minus(v1, v2)                            | v1 - v2                          |                |
| TPE_vec3Cross(v1, v2)                            | Cross product                    |                |
| TPE_vec3Dot(v1, v2)                              | Dot product (scaled by 1/TPE_F)  |                |
| TPE_vec3Len(v)                                   | Exact length                     |                |
| TPE_vec3LenApprox(v)                             | Fast 48-polyhedron approx        |                |
| TPE_vec3Project(v, base)                         | Project v onto base              |                |
| TPE_vec3ProjectNormalized(v, base)               | Project onto normalized base     |                |
| TPE_vec3Times(v, units)                          | Scale by fixed-point             |                |
| TPE_vec3TimesPlain(v, q)                         | Scale by integer                 |                |
| TPE_vec3Normalized(v)                            | Normalize (returns new)          |                |
| TPE_vec3Normalize(&v)                            | Normalize in-place               |                |
| TPE_vec2Angle(x, y)                              | Angle of 2D vector (CCW from +X) |                |
| TPE_sin(x), TPE_cos(x)                           | Trig, x in [0, TPE_F) → 2π       |                |
| TPE_atan(x)                                      | Approximate arctan               |                |
| TPE_sqrt(x)                                      | Integer square root              |                |
| TPE_abs(x), TPE_max(a,b), TPE_min(a,b)           | Standard                         |                |
| TPE_nonZero(x)                                   | Returns x or 1 if x==0           |                |
| TPE_keepInRange(x, min, max)                     | Clamp                            |                |
| TPE_vec3KeepWithinBox(p, center, halfSize)       | Clamp to AABB                    |                |
| TPE_vec3KeepWithinDistanceBand(p, c, minD, maxD) | Clamp distance                   |                |

---

## 7. Joint & Body Creation

|Function|Description|Parameters|
|---|---|---|
|TPE_joint(pos, size)|Create joint|TPE_Vec3 pos, TPE_Unit size|
|TPE_bodyInit(body, joints, jCount, conns, cCount, mass)|Init body||
|TPE_makeBox(joints[8], conns[16], w,d,h, size)|Box (8 corners)||
|TPE_makeCenterBox(joints[9], conns[18], ...)|Box with center joint||
|TPE_makeRect(joints[4], conns[6], w,d, size)|2D rectangle||
|TPE_makeCenterRect(joints[5], conns[8], ...)|Rect + center||
|TPE_makeCenterRectFull(joints[5], conns[10], ...)|Full connections||
|TPE_makeTriangle(joints[3], conns[3], side, size)|Triangle||
|TPE_make2Line(joints[2], conns[1], len, size)|Line segment||

---

## 8. World & Simulation

|Function|Description|
|---|---|
|TPE_worldInit(world, bodies, count, envFunc)|Initialize world|
|TPE_worldStep(world)|Main simulation tick|
|TPE_worldDeactivateAll(world)|Put all bodies to sleep|
|TPE_worldActivateAll(world)|Wake all bodies|
|TPE_worldGetNetSpeed(world)|Total kinetic energy proxy|
|TPE_worldHash(world)|32-bit hash of world state|
|TPE_worldDebugDraw(world, drawFunc, camPos, camRot, camView, gridRes, gridSize)|Debug render|

---

## 9. Body Manipulation

|Function|Description|
|---|---|
|TPE_bodyActivate(body)|Wake up|
|TPE_bodyDeactivate(body)|Put to sleep|
|TPE_bodyIsActive(body)|Check if active|
|TPE_bodyStop(body)|Zero all velocities|
|TPE_bodyMoveBy(body, offset)|Translate|
|TPE_bodyMoveTo(body, pos)|Move center of mass|
|TPE_bodyAccelerate(body, vel)|Add linear velocity|
|TPE_bodyApplyGravity(body, accel)|Add downward acceleration|
|TPE_bodySpin(body, rot)|Add angular velocity|
|TPE_bodySpinWithCenter(body, rot, center)|Spin about custom point|
|TPE_bodyRotateByAxis(body, axisAngle)|Instant rotation|
|TPE_bodyGetCenterOfMass(body)|COM position|
|TPE_bodyGetLinearVelocity(body)|Average velocity|
|TPE_bodyGetAverageSpeed(body)|Avg speed|
|TPE_bodyGetNetSpeed(body)|Total speed (sum of abs velocities)|
|TPE_bodyMultiplyNetSpeed(body, factor)|Scale speed|
|TPE_bodyLimitAverageSpeed(body, min, max)|Clamp speed|
|TPE_bodyGetRotation(body, j1,j2,j3)|Euler angles from 3 joints|
|TPE_bodyGetAABB(body, &min, &max)|Axis-aligned bounding box|
|TPE_bodyGetFastBSphere(body, &c, &r)|Fast bounding sphere|
|TPE_bodyGetBSphere(body, &c, &r)|Tight bounding sphere|
|TPE_bodyReshape(body, env)|Equalize spring tension|
|TPE_bodyCancelOutVelocities(body, strong)|Internal: cancel opposing forces|

---

## 10. Collision & Environment

|Function|Description|
|---|---|
|TPE_bodyEnvironmentCollide(body, env)|Test collision with env|
|TPE_bodyEnvironmentResolveCollision(body, env)|Resolve env collision|
|TPE_jointsResolveCollision(j1, j2, m1, m2, e, f, env)|Resolve joint-joint|
|TPE_jointEnvironmentResolveCollision(joint, e, f, env)|Resolve joint-env|
|TPE_bodiesResolveCollision(b1, b2, env)|Resolve body-body|
|TPE_jointPin(joint, pos)|Fix joint in space|
|TPE_getVelocitiesAfterCollision(&v1,&v2,m1,m2,e)|1D elastic collision|

---

## 11. Ray Casting

| Function                                                           | Description        |
| ------------------------------------------------------------------ | ------------------ |
| TPE_castEnvironmentRay(pos, dir, env, inStep, marchStep, maxSteps) | Ray vs environment |
| TPE_castBodyRay(pos, dir, exclude, world, &bodyIdx, &jointIdx)     | Ray vs bodies      |

---

## 12. Rotation & Orientation

|Function|Description|
|---|---|
|TPE_pointRotate(point, rot)|Rotate point by Euler (ZXY)|
|TPE_rotationInverse(rot)|Inverse Euler rotation|
|TPE_rotationRotateByAxis(rot, axisAngle)|Rotate rotation by axis|
|TPE_rotationFromVecs(forward, right)|Euler from two vectors|
|TPE_fakeSphereRotation(p1, p2, radius)|Fake rolling rotation|

---

## 13. Environment Shape Functions

| Function                                          | Description             |
| ------------------------------------------------- | ----------------------- |
| TPE_envAABox(p, center, maxCorner)                | Axis-aligned box        |
| TPE_envAABoxInside(...)                           | Inside version          |
| TPE_envBox(p, c, max, rot)                        | Rotated box             |
| TPE_envSphere(p, c, r)                            | Sphere                  |
| TPE_envSphereInside(...)                          | Inside sphere           |
| TPE_envHalfPlane(p, c, normal)                    | Infinite plane          |
| TPE_envGround(p, height)                          | Infinite ground         |
| TPE_envInfiniteCylinder(p, c, dir, r)             | Infinite cylinder       |
| TPE_envCylinder(p, c, dir, r)                     | Finite cylinder         |
| TPE_envCone(p, c, dir, r)                         | Cone                    |
| TPE_envLineSegment(p, a, b)                       | Line segment            |
| TPE_envHeightmap(p, c, size, heightFunc, maxDist) | Procedural terrain      |
| TPE_envAATriPrism(p, c, sides[6], depth, dir)     | Triangular prism (ramp) |

---

## 14. Environment Union Helpers (Macros)

c
```c
TPE_ENV_START(test, point)
TPE_ENV_NEXT(test, point)
TPE_ENV_END
TPE_ENV_BCUBE_TEST(...)
TPE_ENV_BSPHERE_TEST(...)
```

Used to combine multiple shapes.

---

## 15. Testing & Debugging

| Function                                                  | Description  |
| --------------------------------------------------------- | ------------ |
| TPE_testClosestPointFunction(f, from, to, res, err, &out) | Validate SDF |
| TPE_jointHash(joint)                                      | 32-bit hash  |
| TPE_connectionHash(conn)                                  |              |
| TPE_bodyHash(body)                                        |              |
| TPE_worldHash(world)                                      |              |

---

## 16. Debug Colors

c
```c
TPE_DEBUG_COLOR_CONNECTION = 0
TPE_DEBUG_COLOR_JOINT = 1
TPE_DEBUG_COLOR_ENVIRONMENT = 2
TPE_DEBUG_COLOR_INACTIVE = 3
```

---

## Summary: Key Workflow

c
```c
// 1. Define environment
TPE_Vec3 myEnv(TPE_Vec3 p, TPE_Unit d) { ... }

// 2. Build bodies
TPE_Joint joints[8];
TPE_Connection conns[16];
TPE_makeBox(joints, conns, 200, 200, 200, 50);

TPE_Body body;
TPE_bodyInit(&body, joints, 8, conns, 16, 1000);

// 3. Init world
TPE_World world;
TPE_worldInit(&world, &body, 1, myEnv);

// 4. Simulate
while (running)
{
  TPE_worldStep(&world);
  render();
}
```

---

License: CC0 1.0 + WAIVER OF ALL IPR  
Author: drummyfish, 2022  
Version: 0.8d Let me know if you want examples, performance tips, or integration with rendering!