#include <PID_v1.h>
#include <Romi32U4.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define ENC_COUNTS_PER_REV 1437.09
#define WHEEL_DIAMETER_M 0.07
#define WHEEL_CIRCUMFERENCE_M WHEEL_DIAMETER_M * PI

#define MAX_WHEEL_VELOCITY 0.75

// PID Constants
const double kP = 1.5;
const double kI = 0;
const double kD = 0;

Romi32U4Motors motors;
Romi32U4Encoders encoders;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

ros::NodeHandle nh;

long lastReadingTime;

double targetVelocity = 0.0;

// PID
double leftSetpoint = 0.0;
double rightSetpoint = 0.0;
double leftInput = 0.0;
double rightInput = 0.0;
double leftOutput = 0.0;
double rightOutput = 0.0;

PID leftPid(&leftInput, &leftOutput, &leftSetpoint, kP, kI, kD, DIRECT);
PID rightPid(&rightInput, &rightOutput, &rightSetpoint, kP, kI, kD, DIRECT);

void cmdvelCb(const geometry_msgs::Twist& cmdvelMsg) {
    nh.loginfo("message received");
}

ros::Subscriber<geometry_msgs::Twist> cmdvelSub("cmd_vel", &cmdvelCb);

double encCountToDistance(int encCount) {
    return (encCount / ENC_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M;
}

double distToVelocity(double distanceM, long timeDeltaMs) {
    double timeDeltaSec = timeDeltaMs / 1000.0;
    return distanceM / timeDeltaSec;
}

void setup()
{
    nh.initNode();
    nh.subscribe(cmdvelSub);
    lastReadingTime = millis();

    leftPid.SetOutputLimits(-MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY);
    rightPid.SetOutputLimits(-MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY);
    leftPid.SetSampleTime(50);
    rightPid.SetSampleTime(50);

    leftPid.SetMode(AUTOMATIC);
    rightPid.SetMode(AUTOMATIC);
}

void loop()
{
    if (buttonA.getSingleDebouncedPress()) {
        targetVelocity = 0.0;
        leftSetpoint = rightSetpoint = targetVelocity;
    }
    else if (buttonB.getSingleDebouncedPress()) {
        targetVelocity -= 0.05;
        if (targetVelocity < -MAX_WHEEL_VELOCITY) {
            targetVelocity = -MAX_WHEEL_VELOCITY;
        }

        leftSetpoint = rightSetpoint = targetVelocity;
    }
    else if (buttonC.getSingleDebouncedPress()) {
        targetVelocity += 0.05;
        if (targetVelocity > MAX_WHEEL_VELOCITY) {
            targetVelocity = MAX_WHEEL_VELOCITY;
        }

        leftSetpoint = rightSetpoint = targetVelocity;
    }

    long currentTime = millis();
    long readTimeDiff = currentTime - lastReadingTime;

    if (readTimeDiff > 20) {
        lastReadingTime = currentTime;
        int16_t leftReading = encoders.getCountsAndResetLeft();
        int16_t rightReading = encoders.getCountsAndResetRight();

        leftInput = distToVelocity(encCountToDistance(leftReading), readTimeDiff);
        rightInput = distToVelocity(encCountToDistance(rightReading), readTimeDiff);

        double leftErr = abs(leftSetpoint - leftInput);
        double rightErr = abs(rightSetpoint - rightInput);

        int leftMotorOutput = ((leftSetpoint + leftOutput) / MAX_WHEEL_VELOCITY) * 300;
        if (leftMotorOutput > 300) leftMotorOutput = 300;
        if (leftMotorOutput < -300) leftMotorOutput = -300;

        int rightMotorOutput = ((rightSetpoint + rightOutput) / MAX_WHEEL_VELOCITY) * 300;
        if (rightMotorOutput > 300) rightMotorOutput = 300;
        if (rightMotorOutput < -300) rightMotorOutput = -300;

        Serial.print("lSet: ");
        Serial.print(leftSetpoint);
        Serial.print(" lIn: ");
        Serial.print(leftInput);
        Serial.print(" lOut: ");
        Serial.print(leftOutput);
        Serial.print(" lMotorOut: ");
        Serial.println(leftMotorOutput);

        motors.setSpeeds(leftMotorOutput, 0);
    }

    leftPid.Compute();
    rightPid.Compute();

    nh.spinOnce();
}
