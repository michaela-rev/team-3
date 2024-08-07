package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Launcher;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_launcher;
  private CANSparkMax m_gatekeeper;

  private boolean m_launcherRunning;
  private boolean m_gatekeeperRunning;
  private double m_gatekeeperPower;
  private double m_launcherPower;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    m_launcher =
        new CANSparkMax(Constants.Launcher.kLaunchCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_launcher.setInverted(false);
    m_launcher.setSmartCurrentLimit(Constants.Launcher.kLaunchCurrentLimit);
    m_launcher.setIdleMode(IdleMode.kBrake);

    m_launcher.burnFlash();

    m_gatekeeper =
        new CANSparkMax(Constants.Launcher.kGateCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_gatekeeper.setInverted(false);
    m_gatekeeper.setSmartCurrentLimit(Constants.Launcher.kGateCurrentLimit);
    m_gatekeeper.setIdleMode(IdleMode.kBrake);

    m_gatekeeper.burnFlash();

    m_launcherRunning = false;
    m_gatekeeperPower = 0.0;
  }

  /**
   * Turns the launcher on. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void runGatekeeper() {
    m_gatekeeperPower = Constants.Launcher.kGatePower;
  }

  public void reverseGatekeeper() {
    m_gatekeeperPower = -Constants.Launcher.kGatePower;
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopGatekeeper() {
    m_gatekeeperPower = 0.0;
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
    // if (m_launcherRunning) {
    //   m_launcher.set(Constants.Launcher.kLaunchPower);
    // } else {
    //   m_launcher.set(0.0);
    // }

    // if (m_gatekeeperRunning) {
    //   m_gatekeeper.set(Constants.Launcher.kGatePower);
    // } else {
    //   m_gatekeeper.set(0.0);
    // }

    m_launcher.set(m_launcherPower);
    m_gatekeeper.set(m_gatekeeperPower);
  }

  /**
   * Turns the intake on. Can be run once and the intake will stay running or run continuously in a
   * {@code RunCommand}.
   */
  public void runLauncher() {
    m_launcherPower = Constants.Launcher.kLaunchPower;
  }

  public void reverseLauncher() {
    m_launcherPower = -0.5;
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopLauncher() {
    m_launcherPower = 0.0;
  }

  public Command primeLauncher() {
    Command newCommand = new Command() {
      private Timer m_timer;

        @Override
        public void initialize() {
          m_timer = new Timer();
          m_timer.start();
        }

        @Override
        public void execute() {
          reverseGatekeeper();
          reverseLauncher();
        }

        @Override
        public boolean isFinished() {
          return m_timer.get() > 0.2;
        }

        @Override
        public void end(boolean interrupted) {
          stopGatekeeper();
          runLauncher();
        }
      };

      newCommand.addRequirements(this);

      return newCommand;
  }
}
