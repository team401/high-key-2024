// package frc.robot.subsystems;

// import com.ctre.phoenix6.Orchestra;
// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.AudioConfigs;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import java.util.List;

// public class OrchestraSubsystem extends SubsystemBase {
//     private Orchestra orchestra = new Orchestra();

//     private StatusCode status;

//     enum PlayState {
//         PLAY,
//         PAUSE,
//         STOP,
//     }

//     String defaultMusic;
//     String currentMusic;

//     PlayState playState = PlayState.STOP;

//     public OrchestraSubsystem(String defaultMusic) {
//         loadMusic(defaultMusic);
//         this.defaultMusic = defaultMusic;

//         SmartDashboard.putBoolean("Orchestra/play", false);
//         SmartDashboard.putString("Orchestra/music", defaultMusic);
//     }

//     AudioConfigs allowMusicDurDisableCfg = new AudioConfigs().withAllowMusicDurDisable(true);

//     @Override
//     public void periodic() {
//         // If robot is enabled, don't waste processing power on any logic.
//         // We're only playing music when robot is disabled anyway
//         if (DriverStation.isEnabled()) {
//             stop();
//             return;
//         }

//         boolean play = SmartDashboard.getBoolean("Orchestra/play", false);
//         String music = SmartDashboard.getString("Orchestra/music", defaultMusic);

//         if (music != currentMusic) {
//             loadMusic(music);
//         }

//         if (play) {
//             play();
//         } else {
//             pause();
//         }
//     }

//     private void logStatusIfNotOk(String whatAreWeDoing) {
//         if (status.isOK()) {
//             System.out.println(
//                     "Orchestra Error While " + whatAreWeDoing + ": " + status.toString());
//         }
//     }

//     public void loadMusic(String musicFilename) {
//         stop();

//         status = orchestra.loadMusic(musicFilename);
//         logStatusIfNotOk("Loading Music File " + musicFilename);

//         currentMusic = musicFilename;
//     }

//     public void addInstrument(TalonFX motor) {
//         status = motor.getConfigurator().apply(allowMusicDurDisableCfg);
//         logStatusIfNotOk("Applying withAllowMusicDurDisable to a motor");
//         orchestra.addInstrument(motor);
//     }

//     public void addInstrumentById(int id) {
//         TalonFX motor = new TalonFX(id);

//         addInstrument(motor);
//     }

//     public void addInstruments(List<TalonFX> motors) {
//         for (TalonFX motor : motors) {
//             addInstrument(motor);
//         }
//     }

//     public void play() {
//         if (playState != PlayState.PLAY && status.isOK()) {
//             status = orchestra.play();
//             logStatusIfNotOk("Playing");

//             playState = PlayState.PLAY;
//         }
//     }

//     public void stop() {
//         if (playState != PlayState.STOP) {
//             status = orchestra.stop();
//             logStatusIfNotOk("Stopping Playback");

//             playState = PlayState.STOP;
//         }
//     }

//     public void pause() {
//         if (playState != PlayState.PAUSE) {
//             status = orchestra.stop();
//             logStatusIfNotOk("Pausing");

//             playState = PlayState.PAUSE;
//         }
//     }

//     public StatusCode getStatus() {
//         return status;
//     }
// }
