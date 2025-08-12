using System.Collections.Generic;

using Godot;

//[GlobalClass]
public partial class CharBody2D : StaticBody2D
{
    public record struct MotionResult(
        Vector2 Travel, 
        Vector2 Remainder, 
        float CollisionSafeFraction,
        float CollisionUnsafeFraction,
        int ColliderShape,
        int CollisionLocalShape,
        GodotObject Collider,
        ulong ColliderId,
        Rid ColliderRid,
        Vector2 ColliderVelocity,
        float CollisionDepth,
        Vector2 CollisionNormal,
        Vector2 CollisionPoint
    )
    {
        public readonly float GetAngle(Vector2 direction)
        {
            return CollisionNormal.Dot(direction);
        }
    }

    public enum MotionModeEnum 
    {
		Grounded,
		Floating,
	};
    public enum PlatformOnLeaveEnum
    {
        Add_Velocity,
        Add_Upward_Velocity,
        Do_Nothing,
    };

    float margin = 0.08f;
    MotionModeEnum motion_mode = MotionModeEnum.Grounded;
    PlatformOnLeaveEnum platform_on_leave = PlatformOnLeaveEnum.Add_Velocity;

    bool floor_constant_speed = false;
    bool floor_stop_on_slope = true;
    bool floor_block_on_wall = true;
    bool slide_on_ceiling = false;
    int max_slides = 4;
    uint platform_layer = 0;
    double floor_max_angle = 0.78539816339744830961566084581988;
    float floor_snap_length = 1;
    double wall_min_slide_angle = 0.26179938779914943653855361527329;
    Vector2 up_direction = Vector2.Up;
    uint platform_floor_layers = uint.MaxValue;
    uint platform_wall_layers = 0;
    Vector2 velocity;

    Vector2 floor_normal;
    Vector2 platform_velocity;
    Vector2 wall_normal;
    Vector2 last_motion;
    Vector2 previous_position;
    Vector2 real_velocity;

    Rid platform_rid;
    ulong platform_object_id;
    bool on_floor = false;
    bool on_ceiling = false;
    bool on_wall = false;

    readonly List<MotionResult> motion_results = [];
    readonly List<KinematicCollision2D> slide_colliders = [];

    const double FLOOR_ANGLE_THRESHOLD = 0.01;

    public int MaxSlides
    {
        get => max_slides;
        set => max_slides = value;
    }
    public Vector2 Velocity
    {
        get => velocity;
        set => velocity = value;
    }
    [Export] public MotionModeEnum MotionMode
    {
        get => motion_mode;
        set => motion_mode = value;
    }
    [Export] public Vector2 UpDirection
    {
        get => up_direction;
        set => up_direction = value == default ? Vector2.Up : value.Normalized();
    }
    [Export] public bool SlideOnCeiling
    {
        get => slide_on_ceiling;
        set => slide_on_ceiling = value;
    }
    [ExportGroup("Floor", "Floor")]
    [Export] public bool FloorStopOnSlope
    {
        get => floor_stop_on_slope;
        set => floor_stop_on_slope = value;
    }
    [Export] public bool FloorConstantSpeed
    {
        get => floor_constant_speed;
        set => floor_constant_speed = value;
    }
    [Export] public bool FloorBlockOnWall
    { 
        get => floor_block_on_wall; 
        set => floor_block_on_wall = value; 
    }
    double max_angle_in_deg = 45;
    [Export(PropertyHint.Range, "0, 180, 0.1,suffix:°")]
    public double FloorMaxAngle 
    { 
        get => max_angle_in_deg;
        set
        {
            max_angle_in_deg = value;
            floor_max_angle = Mathf.DegToRad(max_angle_in_deg);
        } 
    }
    [Export(PropertyHint.Range, "0, 32, 1, suffix:px")]
    public float FloorSnapLength
    {
        get => floor_snap_length;
        set => floor_snap_length = value;
    }
    [ExportGroup("Moving Platform", "Platform")]
    [Export(PropertyHint.Enum, "Add Velocity,Add Upward Velocity,Do Nothing")] 
    public PlatformOnLeaveEnum PlatformOnLeave
    {
        get => platform_on_leave;
        set => platform_on_leave = value;
    }
    [Export(PropertyHint.Layers2DPhysics)] public uint PlatformFloorLayers
    {
        get => platform_floor_layers;
        set => platform_floor_layers = value;
    }
    [Export(PropertyHint.Layers2DPhysics)] public uint PlatformWallLayers
    {
        get => platform_wall_layers;
        set => platform_wall_layers = value;
    }
    [ExportGroup("Collision")]
    [Export(PropertyHint.Range, "0.001, 256, 0.001, suffix:px")] 
    public float SafeMargin
    {
        get => margin;
        set => margin = value;
    }

    private Vector2 GetPositionDelta()
    {
        return GetGlobalTransform().Origin - previous_position;
    }

    private void SetCollisionDirection(ref MotionResult p_result)
    {
        if (motion_mode == MotionModeEnum.Grounded && Mathf.Acos(p_result.GetAngle(up_direction)) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD)
        { //floor
            on_floor = true;
            floor_normal = p_result.CollisionNormal;
            SetPlatformData(ref p_result);
        }
        else if (motion_mode == MotionModeEnum.Grounded && Mathf.Acos(p_result.GetAngle(-up_direction)) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD)
        { //ceiling
            on_ceiling = true;
        }
        else
        {
            on_wall = true;
            wall_normal = p_result.CollisionNormal;
            // Don't apply wall velocity when the collider is a CharacterBody2D.
            
            if (InstanceFromId(p_result.ColliderId) is not CharacterBody2D)
            {
                SetPlatformData(ref p_result);
            }
        }
    }

    private void SetPlatformData(ref MotionResult p_result)
    {
        platform_rid = p_result.ColliderRid;
        platform_object_id = p_result.ColliderId;
        platform_velocity = p_result.ColliderVelocity;
        platform_layer = PhysicsServer2D.BodyGetCollisionLayer(platform_rid);
    }

    private void ApplyFloorSnap(bool p_wall_as_floor)
    {
        if (on_floor)
        {
            return;
        }

        // Snap by at least collision margin to keep floor state consistent.
        float length = Mathf.Max(floor_snap_length, margin);

        PhysicsTestMotionParameters2D parameters = new() 
        {
            From = GetGlobalTransform(),
            Motion = -up_direction * length,
            Margin = margin,
            RecoveryAsCollision = true, // Also report collisions generated only from recovery.
            CollideSeparationRay = true,
        };

        MotionResult result = default;
        if (MoveAndCollide(parameters, ref result, true, false))
        {
            if ((result.GetAngle(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD) ||
                (p_wall_as_floor && result.GetAngle(-up_direction) > floor_max_angle + FLOOR_ANGLE_THRESHOLD))
            {
                on_floor = true;
                floor_normal = result.CollisionNormal;
                SetPlatformData(ref result);

                // Ensure that we only move the body along the up axis
                if (result.Travel.Length() > margin)
                {
                    result.Travel = up_direction * up_direction.Dot(result.Travel);
                }
                else
                {
                    result.Travel = Vector2.Zero;
                }

                var from = parameters.From;
                from.Origin += result.Travel;
                parameters.From = from;
                SetGlobalTransform(parameters.From);
            }
        }
    }

    private void SnapOnFloor(bool p_was_on_floor, bool p_vel_dir_facing_up, bool p_wall_as_floor = false)
    {
        if (on_floor || !p_was_on_floor || p_vel_dir_facing_up)
        {
            return;
        }

        ApplyFloorSnap(p_wall_as_floor);
    }

    private bool OnFloorIfSnapped(bool p_was_on_floor, bool p_vel_dir_facing_up)
    {
        if (up_direction == Vector2.Zero || on_floor || !p_was_on_floor || p_vel_dir_facing_up)
        {
            return false;
        }

        // Snap by at least collision margin to keep floor state consistent.
        float length = Mathf.Max(floor_snap_length, margin);

        PhysicsTestMotionParameters2D parameters = new()
        {
            From = GetGlobalTransform(),
            Motion = -up_direction * length,
            Margin = margin,
            RecoveryAsCollision = true,
            CollideSeparationRay = true,
        };

        MotionResult result = default;
        if (MoveAndCollide(parameters, ref result, true, false))
        {
            if (result.GetAngle(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD)
            {
                return true;
            }
        }

        return false;
    }

    private bool MoveAndCollide(PhysicsTestMotionParameters2D p_parameters, ref MotionResult result, bool p_test_only, bool p_cancel_sliding)
    {
        //if (IsOnlyUpdateTransformChangesEnabled())
        //{
        //    GD.PushError("Move functions do not work together with 'sync to physics' option. See the documentation for details.");
        //}

        PhysicsTestMotionResult2D r_result = new();

        bool colliding = PhysicsServer2D.BodyTestMotion(GetRid(), p_parameters, r_result);
        Vector2 travel = r_result.GetTravel();
        Vector2 remainder = r_result.GetRemainder();
        float collision_safe_fraction = r_result.GetCollisionSafeFraction();
        float collision_unsafe_fraction = r_result.GetCollisionUnsafeFraction();
        int collider_shape = r_result.GetColliderShape();
        int collision_local_shape = r_result.GetCollisionLocalShape();
        GodotObject collider = r_result.GetCollider();
        ulong collider_id = r_result.GetColliderId();
        Rid collider_rid = r_result.GetColliderRid();
        Vector2 collider_velocity = r_result.GetColliderVelocity();
        float collision_depth = r_result.GetCollisionDepth();
        Vector2 collision_normal = r_result.GetCollisionNormal();
        Vector2 collision_point = r_result.GetCollisionPoint();
        // Restore direction of motion to be along original motion,
        // in order to avoid sliding due to recovery,
        // but only if collision depth is low enough to avoid tunneling.
        if (p_cancel_sliding)
        {
            float motion_length = p_parameters.Motion.Length();
            float precision = 0.001f;

            if (colliding)
            {
                // Can't just use margin as a threshold because collision depth is calculated on unsafe motion,
                // so even in normal resting cases the depth can be a bit more than the margin.
                precision += motion_length * (collision_unsafe_fraction - collision_safe_fraction);

                if (collision_depth > p_parameters.Margin + precision)
                {
                    p_cancel_sliding = false;
                }
            }

            if (p_cancel_sliding)
            {
                // When motion is null, recovery is the resulting motion.
                Vector2 motion_normal = default;
                if (motion_length > Mathf.Epsilon)
                {
                    motion_normal = p_parameters.Motion / motion_length;
                }

                // Check depth of recovery.
                float projected_length = travel.Dot(motion_normal);
                Vector2 recovery = travel - motion_normal * projected_length;
                float recovery_length = recovery.Length();
                // Fixes cases where canceling slide causes the motion to go too deep into the ground,
                // because we're only taking rest information into account and not general recovery.
                if (recovery_length < p_parameters.Margin + precision)
                {
                    // Apply adjustment to motion.
                    travel = motion_normal * projected_length;
                    remainder = p_parameters.Motion - travel;
                }
            }
        }

        if (!p_test_only)
        {
            Transform2D gt = p_parameters.From;
            gt.Origin += travel;
            SetGlobalTransform(gt);
        }

        result.Collider = collider;
        result.ColliderRid = collider_rid;
        result.ColliderId = collider_id;
        result.CollisionDepth = collision_depth;
        result.Remainder = remainder;
        result.ColliderShape = collider_shape;
        result.ColliderVelocity = collider_velocity;
        result.CollisionLocalShape = collision_local_shape;
        result.CollisionNormal = collision_normal;
        result.CollisionPoint = collision_point;
        result.CollisionSafeFraction = collision_safe_fraction;
        result.CollisionUnsafeFraction = collision_unsafe_fraction;
        result.Travel = travel;

        return colliding;
    }

    public bool MoveAndSlide(double delta = -1)
    {
        if (delta < 0)
            delta = Engine.IsInPhysicsFrame() ? GetPhysicsProcessDeltaTime() : GetProcessDeltaTime();

        Vector2 current_platform_velocity = platform_velocity;
        Transform2D gt = GetGlobalTransform();
        previous_position = gt.Origin;

        if ((on_floor || on_wall) && platform_rid.IsValid)
        {
            bool excluded = false;
            if (on_floor)
            {
                excluded = (platform_floor_layers & platform_layer) == 0;
            }
            else if (on_wall)
            {
                excluded = (platform_wall_layers & platform_layer) == 0;
            }
            if (!excluded)
            {
                //this approach makes sure there is less delay between the actual body velocity and the one we saved
                PhysicsDirectBodyState2D bs = PhysicsServer2D.BodyGetDirectState(platform_rid);
                if (bs != null)
                {
                    Vector2 local_position = gt.Origin - bs.GetTransform().Origin;
                    current_platform_velocity = bs.GetVelocityAtLocalPosition(local_position);
                }
                else
                {
                    // Body is removed or destroyed, invalidate floor.
                    current_platform_velocity = Vector2.Zero;
                    platform_rid = new Rid();
                }
            }
            else
            {
                current_platform_velocity = Vector2.Zero;
            }
        }

        motion_results.Clear();
        last_motion = Vector2.Zero;

        bool was_on_floor = on_floor;
        on_floor = false;
        on_ceiling = false;
        on_wall = false;

        if (!current_platform_velocity.IsZeroApprox())
        {
            PhysicsTestMotionParameters2D parameters = new()
            {
                From = GetGlobalTransform(),
                Motion = current_platform_velocity * (float)delta,
                Margin = margin,
                RecoveryAsCollision = true,
                ExcludeBodies = [platform_rid],
                ExcludeObjects = platform_object_id != default ? [(int)platform_object_id] : [],
            };

            MotionResult floor_result = new();
            if (MoveAndCollide(parameters, ref floor_result, false, false))
            {
                motion_results.Add(floor_result);
                SetCollisionDirection(ref floor_result);
            }
        }

        if (motion_mode == MotionModeEnum.Grounded)
        {
            MoveAndSlideGrounded(delta, was_on_floor);
        }
        else
        {
            MoveAndSlideFloating(delta);
        }

        // Compute real velocity.
        real_velocity = GetPositionDelta() / (float)delta;

        if (platform_on_leave != PlatformOnLeaveEnum.Do_Nothing)
        {
            // Add last platform velocity when just left a moving platform.
            if (!on_floor && !on_wall)
            {
                if (platform_on_leave == PlatformOnLeaveEnum.Add_Upward_Velocity && current_platform_velocity.Dot(up_direction) < 0)
                {
                    current_platform_velocity = current_platform_velocity.Slide(up_direction);
                }
                velocity += current_platform_velocity;
            }
        }

        return motion_results.Count > 0;
    }

    private void MoveAndSlideFloating(double p_delta)
    {
        Vector2 motion = velocity * (float)p_delta;

        platform_rid = new Rid();
        platform_object_id = default;
        floor_normal = Vector2.Zero;
        platform_velocity = Vector2.Zero;

        bool first_slide = true;
        for (int iteration = 0; iteration < max_slides; ++iteration)
        {
            PhysicsTestMotionParameters2D parameters = new()
            {
                From = GetGlobalTransform(),
                Motion = motion,
                Margin = margin,
                RecoveryAsCollision = true, // Also report collisions generated only from recovery.
            };

            MotionResult result = default;
            bool collided = MoveAndCollide(parameters, ref result, false, false);

            last_motion = result.Travel;

            if (collided)
            {
                motion_results.Add(result);
                SetCollisionDirection(ref result);

                if (result.Remainder.IsZeroApprox())
                {
                    motion = Vector2.Zero; // keep same with cpp source
                    break;  
                }

                if (wall_min_slide_angle != 0 && result.GetAngle(-velocity.Normalized()) < wall_min_slide_angle + FLOOR_ANGLE_THRESHOLD)
                {
                    motion = Vector2.Zero;
                }
                else if (first_slide)
                {
                    Vector2 motion_slide_norm = result.Remainder.Slide(result.CollisionNormal).Normalized();
                    motion = motion_slide_norm * (motion.Length() - result.Travel.Length());
                }
                else
                {
                    motion = result.Remainder.Slide(result.CollisionNormal);
                }

                if (motion.Dot(velocity) <= 0.0f)
                {
                    motion = Vector2.Zero;
                }
            }

            if (!collided || motion.IsZeroApprox())
            {
                break;
            }

            first_slide = false;
        }
    }

    private void MoveAndSlideGrounded(double p_delta, bool p_was_on_floor)
    {
        Vector2 motion = velocity * (float)p_delta;
        Vector2 motion_slide_up = motion.Slide(up_direction);

        Vector2 prev_floor_normal = floor_normal;

        platform_rid = new Rid();
        platform_object_id = default;
        floor_normal = Vector2.Zero;
        platform_velocity = Vector2.Zero;

        bool sliding_enabled = !floor_stop_on_slope;
        bool can_apply_constant_speed = sliding_enabled;
        bool apply_ceiling_velocity = false;
        bool first_slide = true;
        bool vel_dir_facing_up = velocity.Dot(up_direction) > 0;
        Vector2 last_travel = Vector2.Zero;

        for (int iteration = 0; iteration < max_slides; ++iteration)
        {
            PhysicsTestMotionParameters2D parameters = new()
            {
                From = GetGlobalTransform(),
                Motion = motion,
                Margin = margin,
                RecoveryAsCollision = true,
            };

            Vector2 prev_position = parameters.From.Origin;

            MotionResult result = default;
            bool collided = MoveAndCollide(parameters, ref result, false, !sliding_enabled);

            last_motion = result.Travel;

            if (collided)
            {
                motion_results.Add(result);
                SetCollisionDirection(ref result);

                if (on_ceiling && result.ColliderVelocity != Vector2.Zero && result.ColliderVelocity.Dot(up_direction) < 0)
                {
                    if (!slide_on_ceiling || motion.Dot(up_direction) < 0 || (result.CollisionNormal + up_direction).Length() < 0.01f)
                    {
                        apply_ceiling_velocity = true;
                        Vector2 ceiling_vertical_velocity = up_direction * up_direction.Dot(result.ColliderVelocity);
                        Vector2 motion_vertical_velocity = up_direction * up_direction.Dot(velocity);
                        if (motion_vertical_velocity.Dot(up_direction) > 0 || ceiling_vertical_velocity.LengthSquared() > motion_vertical_velocity.LengthSquared())
                        {
                            velocity = ceiling_vertical_velocity + velocity.Slide(up_direction);
                        }
                    }
                }

                if (on_floor && floor_stop_on_slope && (velocity.Normalized() + up_direction).Length() < 0.01f)
                {
                    Transform2D gt = GetGlobalTransform();
                    if (result.Travel.Length() <= margin + Mathf.Epsilon)
                    {
                        gt.Origin -= result.Travel;
                    }
                    SetGlobalTransform(gt);
                    velocity = Vector2.Zero;
                    last_motion = Vector2.Zero;
                    motion = Vector2.Zero; // keep
                    break;
                }

                if (result.Remainder.IsZeroApprox())
                {
                    motion = Vector2.Zero; // keep
                    break;
                }

                if (floor_block_on_wall && on_wall && motion_slide_up.Dot(result.CollisionNormal) <= 0)
                {
                    if (p_was_on_floor && !on_floor && !vel_dir_facing_up)
                    {
                        if (result.Travel.Length() <= margin + Mathf.Epsilon)
                        {
                            Transform2D gt = GetGlobalTransform();
                            gt.Origin -= result.Travel;
                            SetGlobalTransform(gt);
                        }
                        SnapOnFloor(true, false, true);
                        velocity = Vector2.Zero;
                        last_motion = Vector2.Zero;
                        motion = Vector2.Zero; // keep
                        break;
                    }
                    else if (!on_floor)
                    {
                        motion = up_direction * up_direction.Dot(result.Remainder);
                        motion = motion.Slide(result.CollisionNormal);
                    }
                    else
                    {
                        motion = result.Remainder;
                    }
                }
                else if (floor_constant_speed && IsOnFloorOnly() && can_apply_constant_speed && p_was_on_floor && motion.Dot(result.CollisionNormal) < 0)
                {
                    can_apply_constant_speed = false;
                    Vector2 motion_slide_norm = result.Remainder.Slide(result.CollisionNormal).Normalized();
                    motion = motion_slide_norm * (motion_slide_up.Length() - result.Travel.Slide(up_direction).Length() - last_travel.Slide(up_direction).Length());
                }
                else if ((sliding_enabled || !on_floor) && (!on_ceiling || slide_on_ceiling || !vel_dir_facing_up) && !apply_ceiling_velocity)
                {
                    Vector2 slide_motion = result.Remainder.Slide(result.CollisionNormal);
                    if (slide_motion.Dot(velocity) > 0.0f)
                    {
                        motion = slide_motion;
                    }
                    else
                    {
                        motion = Vector2.Zero;
                    }
                    if (slide_on_ceiling && on_ceiling)
                    {
                        if (vel_dir_facing_up)
                        {
                            velocity = velocity.Slide(result.CollisionNormal);
                        }
                        else
                        {
                            velocity = up_direction * up_direction.Dot(velocity);
                        }
                    }
                }
                else
                {
                    motion = result.Remainder;
                    if (on_ceiling && !slide_on_ceiling && vel_dir_facing_up)
                    {
                        velocity = velocity.Slide(up_direction);
                        motion = motion.Slide(up_direction);
                    }
                }

                last_travel = result.Travel;
            }
            else if (floor_constant_speed && first_slide && OnFloorIfSnapped(p_was_on_floor, vel_dir_facing_up))
            {
                can_apply_constant_speed = false;
                sliding_enabled = true;
                Transform2D gt = GetGlobalTransform();
                gt.Origin = prev_position;
                SetGlobalTransform(gt);

                Vector2 motion_slide_norm = motion.Slide(prev_floor_normal).Normalized();
                motion = motion_slide_norm * motion_slide_up.Length();
                collided = true;
            }

            can_apply_constant_speed = !can_apply_constant_speed && !sliding_enabled;
            sliding_enabled = true;
            first_slide = false;

            if (!collided || motion.IsZeroApprox())
            {
                break;
            }
        }

        SnapOnFloor(p_was_on_floor, vel_dir_facing_up);

        if (IsOnWallOnly() && motion_slide_up.Dot(motion_results[0].CollisionNormal) < 0)
        {
            Vector2 slide_motion = velocity.Slide(motion_results[0].CollisionNormal);
            if (motion_slide_up.Dot(slide_motion) < 0)
            {
                velocity = up_direction * up_direction.Dot(velocity);
            }
            else
            {
                velocity = up_direction * up_direction.Dot(velocity) + slide_motion.Slide(up_direction);
            }
        }

        if (on_floor && !vel_dir_facing_up)
        {
            velocity = velocity.Slide(up_direction);
        }
    }

    public bool IsOnFloor()
    {
        return on_floor;
    }

    public bool IsOnFloorOnly()
    {
        return on_floor && !on_wall && !on_ceiling;
    }

    public bool IsOnWall()
    {
        return on_wall;
    }

    public bool IsOnWallOnly()
    {
        return on_wall && !on_floor && !on_ceiling;
    }

    public bool IsOnCeiling()
    {
        return on_ceiling;
    }

    public bool IsOnCeilingOnly()
    {
        return on_ceiling && !on_floor && !on_wall;
    }

    public Vector2 GetFloorNormal()
    {
        return floor_normal;
    }

    public Vector2 GetLastMotion()
    {
        return last_motion;
    }

    public Vector2 GetPlatformVelocity()
    {
        return platform_velocity;
    }

    public Vector2 GetRealVelocity()
    {
        return real_velocity;
    }

    public Vector2 GetWallNormal()
    {
        return wall_normal;
    }

    public int GetSlideCollisionCount()
    {
        return motion_results.Count;
    }

    public MotionResult GetSlideCollision(int bounce)
    {
        return motion_results[bounce];
    }

    public MotionResult GetLastSlideCollision()
    {
        if (motion_results.Count == 0) return new();
        return motion_results[^1];
    }
}
