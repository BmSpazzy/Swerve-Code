
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX steeringMotor;
    private CANCoder encoder;
    
    public SwerveModule(WPI_TalonFX driveMotor,WPI_TalonFX steeringMotor,CANCoder encoder){
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.encoder = encoder;
    
        this.driveMotor.configFactoryDefault();
        this.driveMotor.config_kP(0,0);
        this.driveMotor.config_kI(0,0);
        this.driveMotor.config_kD(0,0);
        this.driveMotor.config_kF(0, 0);
        this.driveMotor.configNominalOutputForward(0);
        this.driveMotor.configNominalOutputReverse(0);
        this.driveMotor.configPeakOutputForward(1);
        this.driveMotor.configPeakOutputReverse(-1);
        this.driveMotor.setSelectedSensorPosition(0);
    
        this.steeringMotor.configFactoryDefault();
        this.steeringMotor.config_kP(0,0);
        this.steeringMotor.config_kI(0,0);
        this.steeringMotor.config_kD(0,0);
        this.steeringMotor.config_kF(0, 0);
        this.steeringMotor.configNominalOutputForward(0);
        this.steeringMotor.configNominalOutputReverse(0);
        this.steeringMotor.configPeakOutputForward(1);
        this.steeringMotor.configPeakOutputReverse(-1);
        this.steeringMotor.setSelectedSensorPosition((this.encoder.getAbsolutePosition()/360/Constants.STEERING_GR)*Constants.PPR);
    }
    public double getVelocityRPM(){
        return driveMotor.getSelectedSensorVelocity()*600/Constants.PPR/Constants.DRIVE_GR;
    }
    public double getAngleDegrees(){
        return steeringMotor.getSelectedSensorPosition()*360/Constants.PPR/Constants.STEERING_GR;
    }
    public void stop(){
        steeringMotor.stopMotor();
        driveMotor.stopMotor();
    }
    public void setDesiredState(SwerveModuleState state){
        SwerveModuleState optimizedState = optimize(state,new Rotation2d(Math.toRadians(getAngleDegrees())));
        double vel = (optimizedState.speedMetersPerSecond/(Constants.WHEEL_RADIUS_METERS*2*Math.PI))*Constants.PPR*Constants.DRIVE_GR*10;
        double pos = (optimizedState.angle.getDegrees()/360)*Constants.PPR*Constants.STEERING_GR;
        steeringMotor.set(ControlMode.Position,pos);
        driveMotor.set(ControlMode.Velocity,vel);
    }
    public static SwerveModuleState optimize(
        SwerveModuleState desiredState, Rotation2d currentAngle) {
      double targetAngle =
          placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
      double targetSpeed = desiredState.speedMetersPerSecond;
      double delta = targetAngle - currentAngle.getDegrees();
      if (Math.abs(delta) > 90) {
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
      }
      return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360 - lowerOffset);
      } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
        newAngle += 360;
      }
      while (newAngle > upperBound) {
        newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
        newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
        newAngle += 360;
      }
      return newAngle;
    }

    public void move(double driveVelocity, double steeringVelocity){
        driveMotor.set(ControlMode.PercentOutput,driveVelocity);
        steeringMotor.set(ControlMode.PercentOutput,steeringVelocity);
    }

}
