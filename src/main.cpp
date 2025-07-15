#include <mujoco/mujoco.h>
#include <mujoco.h>
#include <iostream>
#include <mujoco_viz.h>
#include <stdio.h>
#include <math.h>

#define PI 3.141592653589793
constexpr double TOLERANCE_ARM = 0.01; // rad
constexpr double TOLERANCE_LIFT = 0.01; // m
constexpr double TOLERANCE_GRIPPER = 0.001; // N

constexpr double GRIPPER_FORCE = 10.0; // N

// Enumeration for profile shapes
enum ProfileShape {
    PROFILE_TRAPEZOIDAL,
    PROFILE_TRIANGULAR
};

// Enumeration for robot status
enum RobotStatus {
    STATUS_IDLE,
    STATUS_ARM_MOVING,
    STATUS_LIFT_MOVING,
    STATUS_GRIPPER_MOVING
};

// Class representing a trapezoidal velocity profile
class VelocityProfile {
public:
    double max_velocity;
    double max_acceleration;
    double peak_velocity;

    double accel_time;
    double cruise_time;
    double decel_time;
    double total_time;

    double accel_angle;
    double cruise_angle;
    double decel_angle;

    double direction;

    ProfileShape shape;

    void initialize(double start_angle, double end_angle, double vmax, double amax) {
        double angle_diff = fabs(end_angle - start_angle);
        direction = (end_angle > start_angle) ? 1.0 : -1.0;
        max_velocity = vmax;
        max_acceleration = amax;

        accel_angle = (vmax * vmax) / (2.0 * amax);
        decel_angle = accel_angle;

        if (angle_diff >= (accel_angle + decel_angle)) {
            shape = PROFILE_TRAPEZOIDAL;
            accel_time = vmax / amax;
            decel_time = accel_time;
            cruise_angle = angle_diff - (accel_angle + decel_angle);
            cruise_time = cruise_angle / vmax;
            peak_velocity = vmax;
        } else {
            shape = PROFILE_TRIANGULAR;
            peak_velocity = sqrt(angle_diff * amax);
            accel_time = peak_velocity / amax;
            decel_time = accel_time;
            cruise_time = 0.0;
        }
        total_time = accel_time + cruise_time + decel_time;
    }

    double get_velocity(double elapsed_time) const {
        if (elapsed_time <= 0.0 || elapsed_time >= total_time) return 0.0;

        if (shape == PROFILE_TRAPEZOIDAL) {
            if (elapsed_time < accel_time) {
                return direction * max_acceleration * elapsed_time;
            } else if (elapsed_time < (accel_time + cruise_time)) {
                return direction * peak_velocity;
            } else {
                double decel_elapsed = elapsed_time - accel_time - cruise_time;
                return direction * (peak_velocity - max_acceleration * decel_elapsed);
            }
        } else {
            if (elapsed_time < accel_time) {
                return direction * max_acceleration * elapsed_time;
            } else {
                double decel_elapsed = elapsed_time - accel_time;
                return direction * (peak_velocity - max_acceleration * decel_elapsed);
            }
        }
    }
};

// Class representing the SCARA robot arm
class ScaraArm {
public:
    double arm_length_1;
    double arm_length_2;

    double current_angle_1;
    double current_angle_2;
    double target_angle_1;
    double target_angle_2;

    double max_velocity;
    double max_acceleration;

    RobotStatus status;
    double target_height = 0.0;
    double target_gripper = -GRIPPER_FORCE;
    double start_time;
    uint8_t motion_completed_1;
    uint8_t motion_completed_2;
    double current_height; 
    double gripper_time;
    double target_gripper_angle;

    VelocityProfile profile_1;
    VelocityProfile profile_2;

    void initialize() {
        arm_length_1 = 100.0;
        arm_length_2 = 100.0;
        status = STATUS_IDLE;
        max_velocity = 2.0;
        max_acceleration = 1.0;
    }

    void update_sensor_readings(double angle_1, double angle_2, double z_height) {
        current_height = z_height;
        current_angle_1 = angle_1;
        current_angle_2 = angle_2;
    }

    void compute_inverse_kinematics(double x, double y) {
        double distance = sqrt(x * x + y * y);
        double alpha = atan2(y, x);
        double beta = acos((arm_length_1 * arm_length_1 - arm_length_2 * arm_length_2 + distance * distance) / (2.0 * arm_length_1 * distance));

        target_angle_1 = alpha + beta;
        target_angle_2 = acos((arm_length_1 * arm_length_1 + arm_length_2 * arm_length_2 - distance * distance) / (2.0 * arm_length_1 * arm_length_2));
    }

    void plan_trajectory(double current_time) {
        profile_1.initialize(current_angle_1, target_angle_1, max_velocity, max_acceleration);
        profile_2.initialize(current_angle_2, target_angle_2, max_velocity, max_acceleration);
        start_time = current_time;
        motion_completed_1 = 0;
        motion_completed_2 = 0;
    }

    void execute_trajectory(mjData* d) {
        // Always apply height and gripper commands
        d->ctrl[0] = target_height;
        d->ctrl[4] = target_gripper;
        d->ctrl[3] = target_gripper_angle;
        
        // Only apply arm velocities if moving arms
        if (status == STATUS_ARM_MOVING) {
            double elapsed_time = d->time - start_time;
            double velocity_1 = profile_1.get_velocity(elapsed_time);
            double velocity_2 = profile_2.get_velocity(elapsed_time);

            if (fabs(current_angle_1 - target_angle_1) < TOLERANCE_ARM) { 
                velocity_1 = 0.0; 
                motion_completed_1 = 1; 
            }
            if (fabs(current_angle_2 - target_angle_2) < TOLERANCE_ARM) { 
                velocity_2 = 0.0; 
                motion_completed_2 = 1; 
            }
            
            d->ctrl[1] = velocity_1;
            d->ctrl[2] = velocity_2;
        }
    }
    
    bool is_motion_completed() {
        return (motion_completed_1 == 1 && motion_completed_2 == 1 );
    }
    
    void move_to(double x, double y, double grip_angle, double current_time) {
        compute_inverse_kinematics(x, y);
        plan_trajectory(current_time);
        target_gripper_angle = grip_angle;
        status = STATUS_ARM_MOVING;
        printf("Trajectory planned and running.\n");
    }

    void set_height(double value) {
        target_height = value;
        status = STATUS_LIFT_MOVING;
    }

    void close_gripper(double current_time) {
        target_gripper = -GRIPPER_FORCE;
        status = STATUS_GRIPPER_MOVING;
        gripper_time = current_time;
    }

    void open_gripper(double current_time) {
        target_gripper = GRIPPER_FORCE;
        status = STATUS_GRIPPER_MOVING;
        gripper_time = current_time;
    }
    
    bool check_movement_completion(double current_time) {
        switch (status) {
            case STATUS_ARM_MOVING:
                if (motion_completed_1 && motion_completed_2) {
                    status = STATUS_IDLE;
                    printf("Arm movement completed\n");
                    return true;
                }
                break;
                
            case STATUS_LIFT_MOVING:
                
                if (fabs(current_height - target_height) < TOLERANCE_LIFT) {
                    status = STATUS_IDLE;
                    printf("Lift movement completed\n");
                    return true;
                }
                break;
                
            case STATUS_GRIPPER_MOVING:
                if (current_time - gripper_time > 1.0) {
                    status = STATUS_IDLE;
                    printf("Gripper movement completed\n");
                    return true;
                }
                break;
                
            case STATUS_IDLE:
                return true;
        }
        return false;
    }
};

ScaraArm scara;
int step = 0, mission = 0, last_step = -1;
double x_targets[4] = {100.0, -100.0, 0.0, 0.0};
double y_targets[4] = {100.0, 100.0, 150.0, -150.0};
double gripper_rot_targets[4] = {0.0, 0.0, 0.78539, 0.78539};

// Main control loop
void controllers(const mjModel* model, mjData* data) {
    scara.update_sensor_readings(data->sensordata[1], data->sensordata[2], data->sensordata[0]);
    
    if(data->time > 10.0){
    if (mission < 4) {
        if (step != last_step) {
            // First entry in this step
            last_step = step;
            switch (step) {
                case 0:
                    scara.open_gripper(data->time);
                    break;
                case 1:
                    scara.set_height(0.021*(mission+1));
                    break;
                case 2: 
                    scara.move_to(x_targets[mission], y_targets[mission], gripper_rot_targets[mission], data->time);
                    break;
                case 3:
                    scara.set_height(0.0);
                    break;
                case 4:
                    scara.close_gripper(data->time);
                    break;
                case 5:
                    scara.set_height(0.021*(mission+1));
                    break;
                case 6:
                    scara.move_to(50.0, 50.0, 0.0, data->time);
                    break;
            }
        } else {
            // Check if current movement completed
            if (scara.check_movement_completion(data->time)) {
                step++;
                
                // After gripper opens, move to next mission
                if (step == 7) {
                    step = 0;
                    mission++;
                    printf("Starting mission %d\n", mission);
                }
            }
        }
    }
    
    else{
        if (step != last_step) {
            // First entry in this step
            last_step = step;
            switch (step) {
                case 0:
                    scara.open_gripper(data->time);
                    break;
                case 1: 
                    scara.move_to(1.0, 1.0, 0.0, data->time);
                    break;
                case 3:
                    scara.set_height(0.0);
                    break;
            }
        } else {
            // Check if current movement completed
            if (scara.check_movement_completion(data->time)) {
                step++;
                
                // After gripper opens, move to next mission
                if (step == 4) {
                    step = 0;
         
                    printf("Starting mission %d\n", mission);
                }
            }
        }
    }
    }
    
    scara.execute_trajectory(d);
}

int main(int argc, const char** argv) {
    const char* xml_filepath = "../xml/world.xml";
    m = mj_loadXML(xml_filepath, NULL, NULL, 0);
    d = mj_makeData(m);

    scara.initialize();
    mjcb_control = controllers;

    mujoco_loop();

    return 0;
}

