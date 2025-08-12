class_name KinematicBody2D
extends StaticBody2D

const CMP_EPSILON := 0.00001

class MotionResult:
	extends RefCounted
	
	var travel: Vector2
	var remainder: Vector2
	var collision_safe_fraction: float
	var collision_unsafe_fraction: float
	var collider_shape: int
	var collision_local_shape: int
	var collider: Object
	var collider_id: int
	var collider_rid: RID
	var collider_velocity: Vector2
	var collision_depth: float
	var collision_normal: Vector2
	var collision_point: Vector2
	
	func get_angle(direction: Vector2) -> float:
		return collision_normal.dot(direction)

enum {
	MOTION_MODE_GROUNDED = 0,
	MOTION_MODE_FLOATING = 1
}

enum {
	PLATFORM_ON_LEAVE_ADD_VELOCITY = 0,
	PLATFORM_ON_LEAVE_ADD_UPWARD_VELOCITY = 1,
	PLATFORM_ON_LEAVE_DO_NOTHING = 2
}

@export_enum("Grounded", "Floating") var motion_mode : int = MOTION_MODE_GROUNDED
@export var up_direction := Vector2.UP:
	set(v):
		up_direction = Vector2.UP if v == Vector2.ZERO else v.normalized()
@export var slide_on_ceiling := true

@export_group("Floor", "floor_")
@export var floor_stop_on_slope := true
@export var floor_constant_speed := false
@export var floor_block_on_wall := true
var floor_max_angle : float = deg_to_rad(45)
@export_range(0, 180, 0.1, "suffix:°") var _floor_max_angle : float = 45.:
	get(): return floor_max_angle
	set(v):
		floor_max_angle = deg_to_rad(v)
@export_range(0, 32, 0.1, "suffix:px") var floor_snap_length : float = 1.
var wall_min_slide_angle := deg_to_rad(15)

@export_group("Moving Platform", "platform_")
@export_enum("Add Velocity", "Add Upward Velocity", "Do Nothing") var platform_on_leave : int = PLATFORM_ON_LEAVE_ADD_VELOCITY
@export_flags_2d_physics var platform_floor_layers : int = -1
@export_flags_2d_physics var platform_wall_layers : int = 0
var _platform_layer := 0

@export_group("Collision")
@export_range(0.001, 256, 0.001, "suffix:px") var safe_margin : float = 0.08

var max_slides := 4
var velocity := Vector2.ZERO
var _floor_normal := Vector2.ZERO
var _platform_velocity := Vector2.ZERO
var _wall_normal := Vector2.ZERO
var _last_motion := Vector2.ZERO
var _previous_position := Vector2.ZERO
var _real_velocity := Vector2.ZERO

var _platform_rid : RID
var _platform_object_id : int
var _on_floor := false;
var _on_ceiling := false;
var _on_wall := false;

var _motion_results : Array[MotionResult] = []

const FLOOR_ANGLE_THRESHOLD := 0.01

func is_on_floor() -> bool:
	return _on_floor

func is_on_floor_only() -> bool:
	return _on_floor and not _on_wall and not _on_ceiling

func is_on_wall() -> bool:
	return _on_wall

func is_on_wall_only() -> bool:
	return _on_wall and not _on_floor and not _on_ceiling

func is_on_ceiling() -> bool:
	return _on_ceiling

func is_on_ceiling_only() -> bool:
	return _on_ceiling and not _on_floor and not _on_wall

func get_floor_normal() -> Vector2:
	return _floor_normal

func get_last_motion() -> Vector2:
	return _last_motion

func get_platform_velocity() -> Vector2:
	return _platform_velocity

func get_position_delta() -> Vector2:
	return global_transform.origin - _previous_position

func get_real_velocity() -> Vector2:
	return _real_velocity

func get_wall_normal() -> Vector2:
	return _wall_normal

func get_slide_collision_count() -> int:
	return _motion_results.size()

func get_slide_collision(p_bounce: int) -> MotionResult:
	return _motion_results[p_bounce]

func get_last_slide_collision() -> MotionResult:
	if _motion_results.is_empty(): return MotionResult.new()
	return _motion_results[_motion_results.size() - 1]

func _set_platform_data(p_result: MotionResult) -> void:
	_platform_rid = p_result.collider_rid
	_platform_object_id = p_result.collider_id
	_platform_velocity = p_result.collider_velocity
	_platform_layer = PhysicsServer2D.body_get_collision_layer(_platform_rid)

func _set_collision_direction(p_result: MotionResult) -> void:
	if motion_mode == MOTION_MODE_GROUNDED and acos(p_result.get_angle(up_direction)) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD:
		# floor
		_on_floor = true
		_floor_normal = p_result.collision_normal
		_set_platform_data(p_result)
	elif motion_mode == MOTION_MODE_GROUNDED and acos(p_result.get_angle(-up_direction)) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD:
		# ceiling
		_on_ceiling = true
	else:
		_on_wall = true
		_wall_normal = p_result.collision_normal
		# Don't apply wall velocity when the collider is a CharacterBody2D.
		if instance_from_id(p_result.collider_id) is not CharacterBody2D:
			_set_platform_data(p_result)

func _move_and_collide(p_parameters: PhysicsTestMotionParameters2D, result: MotionResult, p_test_only: bool, p_cancel_sliding: bool) -> bool:
	#if (is_only_update_transform_changes_enabled()):
	#	push_error("Move functions do not work together with 'sync to physics' option. See the documentation for details.")
	
	var r_result := PhysicsTestMotionResult2D.new()
	
	var colliding := PhysicsServer2D.body_test_motion(get_rid(), p_parameters, r_result)
	var travel := r_result.get_travel()
	var remainder := r_result.get_remainder()
	var collision_safe_fraction := r_result.get_collision_safe_fraction()
	var collision_unsafe_fraction := r_result.get_collision_unsafe_fraction()
	var collider_shape := r_result.get_collider_shape()
	var collision_local_shape := r_result.get_collision_local_shape()
	var collider := r_result.get_collider()
	var collider_id := r_result.get_collider_id()
	var collider_rid := r_result.get_collider_rid()
	var collider_velocity := r_result.get_collider_velocity()
	var collision_depth := r_result.get_collision_depth()
	var collision_normal := r_result.get_collision_normal()
	var collision_point := r_result.get_collision_point()
	
	# Restore direction of motion to be along original motion,
	# in order to avoid sliding due to recovery,
	# but only if collision depth is low enough to avoid tunneling.
	if p_cancel_sliding:
		var motion_length := p_parameters.motion.length()
		var precision := 0.001
		
		if colliding:
			# Can't just use margin as a threshold because collision depth is calculated on unsafe motion,
			# so even in normal resting cases the depth can be a bit more than the margin.
			precision += motion_length * (collision_unsafe_fraction - collision_safe_fraction)
			
			if collision_depth > p_parameters.margin + precision:
				p_cancel_sliding = false
		
		if p_cancel_sliding:
			# When motion is null, recovery is the resulting motion.
			var motion_normal := Vector2.ZERO
			if motion_length > CMP_EPSILON:
				motion_normal = p_parameters.motion / motion_length
			
			# Check depth of recovery.
			var projected_length := travel.dot(motion_normal)
			var recovery := travel - motion_normal * projected_length
			var recovery_length := recovery.length()
			# Fixes cases where canceling slide causes the motion to go too deep into the ground,
			# because we're only taking rest information into account and not general recovery.
			if recovery_length < p_parameters.margin + precision:
				# Apply adjustment to motion.
				travel = motion_normal * projected_length
				remainder = p_parameters.motion - travel
	
	if not p_test_only:
		var gt = p_parameters.from
		gt.origin += travel
		set_global_transform(gt)
	
	result.collider = collider
	result.collider_rid = collider_rid
	result.collider_id = collider_id
	result.collision_depth = collision_depth
	result.remainder = remainder
	result.collider_shape = collider_shape
	result.collider_velocity = collider_velocity
	result.collision_local_shape = collision_local_shape
	result.collision_normal = collision_normal
	result.collision_point = collision_point
	result.collision_safe_fraction = collision_safe_fraction
	result.collision_unsafe_fraction = collision_unsafe_fraction
	result.travel = travel
	
	return colliding

func apply_floor_snap(p_wall_as_floor: bool) -> void:
	if _on_floor:
		return
	
	# Snap by at least collision margin to keep floor state consistent.
	var length = max(floor_snap_length, safe_margin)
	
	var parameters := PhysicsTestMotionParameters2D.new()
	parameters.from = get_global_transform()
	parameters.motion = -up_direction * length
	parameters.margin = safe_margin
	parameters.recovery_as_collision = true  # Also report collisions generated only from recovery.
	parameters.collide_separation_ray = true
	
	var result := MotionResult.new()
	if _move_and_collide(parameters, result, true, false):
		if (result.get_angle(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD) or \
		   (p_wall_as_floor and result.get_angle(-up_direction) > floor_max_angle + FLOOR_ANGLE_THRESHOLD):
			_on_floor = true
			_floor_normal = result.collision_normal
			_set_platform_data(result)
			
			# Ensure that we only move the body along the up axis
			if result.travel.length() > safe_margin:
				result.travel = up_direction * up_direction.dot(result.travel)
			else:
				result.travel = Vector2.ZERO
			
			var from := parameters.from
			from.origin += result.travel
			parameters.from = from
			set_global_transform(parameters.from)

func _snap_on_floor(p_was_on_floor: bool, p_vel_dir_facing_up: bool, p_wall_as_floor := false) -> void:
	if _on_floor or not p_was_on_floor or p_vel_dir_facing_up:
		return
	
	apply_floor_snap(p_wall_as_floor)

func _on_floor_if_snapped(p_was_on_floor: bool, p_vel_dir_facing_up: bool) -> bool:
	if up_direction == Vector2.ZERO or _on_floor or not p_was_on_floor or p_vel_dir_facing_up:
		return false
	
	# Snap by at least collision margin to keep floor state consistent.
	var length = max(floor_snap_length, safe_margin)
	
	var parameters := PhysicsTestMotionParameters2D.new()
	parameters.from = get_global_transform()
	parameters.motion = -up_direction * length
	parameters.margin = safe_margin
	parameters.recovery_as_collision = true
	parameters.collide_separation_ray = true
	
	var result := MotionResult.new()
	if _move_and_collide(parameters, result, true, false):
		if result.get_angle(up_direction) <= floor_max_angle + FLOOR_ANGLE_THRESHOLD:
			return true
	
	return false

func _move_and_slide_floating(p_delta: float) -> void:
	var motion := velocity * p_delta
	
	_platform_rid = RID()
	_platform_object_id = 0
	_floor_normal = Vector2.ZERO
	_platform_velocity = Vector2.ZERO
	
	var first_slide := true
	for iteration in max_slides:
		var parameters := PhysicsTestMotionParameters2D.new()
		parameters.from = get_global_transform()
		parameters.motion = motion
		parameters.margin = safe_margin
		parameters.recovery_as_collision = true  # Also report collisions generated only from recovery.
		
		var result := MotionResult.new()
		var collided := _move_and_collide(parameters, result, false, false)
		
		_last_motion = result.travel
		
		if collided:
			_motion_results.append(result)
			_set_collision_direction(result)
			
			if result.remainder.is_zero_approx():
				motion = Vector2.ZERO  # keep same with cpp source
				break
			
			if wall_min_slide_angle != 0 and result.get_angle(-velocity.normalized()) < wall_min_slide_angle + FLOOR_ANGLE_THRESHOLD:
				motion = Vector2.ZERO
			elif first_slide:
				var motion_slide_norm := result.remainder.slide(result.collision_normal).normalized()
				motion = motion_slide_norm * (motion.length() - result.travel.length())
			else:
				motion = result.remainder.slide(result.collision_normal)
			
			if motion.dot(velocity) <= 0.0:
				motion = Vector2.ZERO
		
		if not collided or motion.is_zero_approx():
			break
		
		first_slide = false

func _move_and_slide_grounded(p_delta: float, p_was_on_floor: bool) -> void:
	var motion := velocity * p_delta
	var motion_slide_up := motion.slide(up_direction)
	
	var prev_floor_normal := _floor_normal
	
	_platform_rid = RID()
	_platform_object_id = 0
	_floor_normal = Vector2.ZERO
	_platform_velocity = Vector2.ZERO
	
	var sliding_enabled := not floor_stop_on_slope
	var can_apply_constant_speed := sliding_enabled
	var apply_ceiling_velocity := false
	var first_slide := true
	var vel_dir_facing_up := velocity.dot(up_direction) > 0
	var last_travel := Vector2.ZERO
	
	for iteration in max_slides:
		var parameters := PhysicsTestMotionParameters2D.new()
		parameters.from = get_global_transform()
		parameters.motion = motion
		parameters.margin = safe_margin
		parameters.recovery_as_collision = true
		
		var prev_position := parameters.from.origin
		
		var result := MotionResult.new()
		var collided := _move_and_collide(parameters, result, false, not sliding_enabled)
		
		_last_motion = result.travel
		
		if collided:
			_motion_results.append(result)
			_set_collision_direction(result)
			
			if _on_ceiling and result.collider_velocity != Vector2.ZERO and result.collider_velocity.dot(up_direction) < 0:
				if not slide_on_ceiling or motion.dot(up_direction) < 0 or (result.collision_normal + up_direction).length() < 0.01:
					apply_ceiling_velocity = true
					var ceiling_vertical_velocity := up_direction * up_direction.dot(result.collider_velocity)
					var motion_vertical_velocity := up_direction * up_direction.dot(velocity)
					if motion_vertical_velocity.dot(up_direction) > 0 or ceiling_vertical_velocity.length_squared() > motion_vertical_velocity.length_squared():
						velocity = ceiling_vertical_velocity + velocity.slide(up_direction)
			
			if _on_floor and floor_stop_on_slope and (velocity.normalized() + up_direction).length() < 0.01:
				var gt := get_global_transform()
				if result.travel.length() <= safe_margin + CMP_EPSILON:
					gt.origin -= result.travel
				set_global_transform(gt)
				velocity = Vector2.ZERO
				_last_motion = Vector2.ZERO
				motion = Vector2.ZERO  # keep
				break
			
			if result.remainder.is_zero_approx():
				motion = Vector2.ZERO  # keep
				break
			
			if floor_block_on_wall and _on_wall and motion_slide_up.dot(result.collision_normal) <= 0:
				if p_was_on_floor and not _on_floor and not vel_dir_facing_up:
					if result.travel.length() <= safe_margin + CMP_EPSILON:
						var gt := get_global_transform()
						gt.origin -= result.travel
						set_global_transform(gt)
					_snap_on_floor(true, false, true)
					velocity = Vector2.ZERO
					_last_motion = Vector2.ZERO
					motion = Vector2.ZERO  # keep
					break
				elif not _on_floor:
					motion = up_direction * up_direction.dot(result.remainder)
					motion = motion.slide(result.collision_normal)
				else:
					motion = result.remainder
			elif floor_constant_speed and is_on_floor_only() and can_apply_constant_speed and p_was_on_floor and motion.dot(result.collision_normal) < 0:
				can_apply_constant_speed = false
				var motion_slide_norm := result.remainder.slide(result.collision_normal).normalized()
				motion = motion_slide_norm * (motion_slide_up.length() - result.travel.slide(up_direction).length() - last_travel.slide(up_direction).length())
			elif (sliding_enabled or not _on_floor) and (not _on_ceiling or slide_on_ceiling or not vel_dir_facing_up) and not apply_ceiling_velocity:
				var slide_motion := result.remainder.slide(result.collision_normal)
				if slide_motion.dot(velocity) > 0.0:
					motion = slide_motion
				else:
					motion = Vector2.ZERO
				if slide_on_ceiling and _on_ceiling:
					if vel_dir_facing_up:
						velocity = velocity.slide(result.collision_normal)
					else:
						velocity = up_direction * up_direction.dot(velocity)
			else:
				motion = result.remainder
				if _on_ceiling and not slide_on_ceiling and vel_dir_facing_up:
					velocity = velocity.slide(up_direction)
					motion = motion.slide(up_direction)
			
			last_travel = result.travel
		elif floor_constant_speed and first_slide and _on_floor_if_snapped(p_was_on_floor, vel_dir_facing_up):
			can_apply_constant_speed = false
			sliding_enabled = true
			var gt := get_global_transform()
			gt.origin = prev_position
			set_global_transform(gt)
			
			var motion_slide_norm := motion.slide(prev_floor_normal).normalized()
			motion = motion_slide_norm * motion_slide_up.length()
			collided = true
		
		can_apply_constant_speed = not can_apply_constant_speed and not sliding_enabled
		sliding_enabled = true
		first_slide = false
		
		if not collided or motion.is_zero_approx():
			break
	
	_snap_on_floor(p_was_on_floor, vel_dir_facing_up)
	
	if is_on_wall_only() and motion_slide_up.dot(_motion_results[0].collision_normal) < 0:
		var slide_motion := velocity.slide(_motion_results[0].collision_normal)
		if motion_slide_up.dot(slide_motion) < 0:
			velocity = up_direction * up_direction.dot(velocity)
		else:
			velocity = up_direction * up_direction.dot(velocity) + slide_motion.slide(up_direction)
	
	if _on_floor and not vel_dir_facing_up:
		velocity = velocity.slide(up_direction)

func move_and_slide(delta: float = -1) -> bool:
	if delta < 0:
		delta = get_physics_process_delta_time() if Engine.is_in_physics_frame() else get_process_delta_time()
	
	var current_platform_velocity := _platform_velocity
	var gt := get_global_transform()
	_previous_position = gt.origin
	
	if (_on_floor or _on_wall) and _platform_rid.is_valid():
		var excluded := false
		if _on_floor:
			excluded = (platform_floor_layers & _platform_layer) == 0
		elif _on_wall:
			excluded = (platform_wall_layers & _platform_layer) == 0
		
		if not excluded:
			# This approach makes sure there is less delay between the actual body velocity and the one we saved
			var bs := PhysicsServer2D.body_get_direct_state(_platform_rid)
			if bs != null:
				var local_position := gt.origin - bs.get_transform().origin
				current_platform_velocity = bs.get_velocity_at_local_position(local_position)
			else:
				# Body is removed or destroyed, invalidate floor
				current_platform_velocity = Vector2.ZERO
				_platform_rid = RID()
		else:
			current_platform_velocity = Vector2.ZERO
	
	_motion_results.clear()
	_last_motion = Vector2.ZERO
	
	var was_on_floor := _on_floor
	_on_floor = false
	_on_ceiling = false
	_on_wall = false
	
	if not current_platform_velocity.is_zero_approx():
		var parameters := PhysicsTestMotionParameters2D.new()
		parameters.from = get_global_transform()
		parameters.motion = current_platform_velocity * delta
		parameters.margin = safe_margin
		parameters.recovery_as_collision = true
		parameters.exclude_bodies = [_platform_rid]
		if _platform_object_id != 0:
			parameters.exclude_objects = [_platform_object_id]
		
		var floor_result := MotionResult.new()
		if _move_and_collide(parameters, floor_result, false, false):
			_motion_results.append(floor_result)
			_set_collision_direction(floor_result)
	
	if motion_mode == MOTION_MODE_GROUNDED:
		_move_and_slide_grounded(delta, was_on_floor)
	else:
		_move_and_slide_floating(delta)
	
	# Compute real velocity
	_real_velocity = get_position_delta() / delta
	
	if platform_on_leave != PLATFORM_ON_LEAVE_DO_NOTHING:
		# Add last platform velocity when just left a moving platform
		if not _on_floor and not _on_wall:
			if platform_on_leave == PLATFORM_ON_LEAVE_ADD_UPWARD_VELOCITY and current_platform_velocity.dot(up_direction) < 0:
				current_platform_velocity = current_platform_velocity.slide(up_direction)
			velocity += current_platform_velocity
	
	return _motion_results.size() > 0
