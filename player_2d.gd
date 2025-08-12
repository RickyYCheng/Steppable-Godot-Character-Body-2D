extends KinematicBody2D


const SPEED = 300.0
const JUMP_VELOCITY = -400.0
const gravity = 980
# ATTENTION:
# get_gravity() returns Vector2.ZERO

func _physics_process(delta: float) -> void:
	# Add the gravity.
	if not is_on_floor():
		velocity += -up_direction * gravity * delta

	# Handle jump.
	if Input.is_action_just_pressed("ui_accept") and is_on_floor():
		velocity.y = JUMP_VELOCITY

	# Get the input direction and handle the movement/deceleration.
	# As good practice, you should replace UI actions with custom gameplay actions.
	var direction := Input.get_axis("ui_left", "ui_right")
	if direction:
		velocity.x = direction * SPEED
	else:
		velocity.x = move_toward(velocity.x, 0, SPEED)
	
	move_and_slide()
	
	prints(is_on_floor(), is_on_wall(), instance_from_id(_platform_object_id))
	
	# default = -1. 
	# will use process_delta by default where delta < 0
	# move_and_slide()
