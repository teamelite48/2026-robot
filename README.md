# 2026 Robot

## General Reminders

- At the end of the season or beginning of the next, remember to update the visibility of this repo to **Public**.
- Remember to update [Elastic.java](./src/main/java/frc/robot/util/Elastic.java) (source: [Elastic](https://github.com/Gold872/elastic_dashboard/releases)) & [LimelightHelpers.java](./src/main/java/frc/robot/lib/LimelightHelpers.java) (source: [Limelight](https://docs.limelightvision.io/docs/resources/downloads)) packages from the source.
- Update offsets anytime something changes mechanically.
  - Changing wheels counts as a valid time to update the [DriveConfig.java](./src/main/java/frc/robot/subsystems/drive/DriveConfig.java) offsets.
- No magic numbers! Create a variable with a meaningful name and add it to a config file.
- All robot based units in configs should be based in inches. Convert to metric as needed (especially for pose estimation).
  - A conversion function can be found in [EliteMath.java](./src/main/java/frc/robot/util/EliteMath.java).
- Remember to wipe Path Planner paths in [/deploy/pathplanner/](./src/main/deploy/pathplanner/) from the previous year.
  - If there are autos already deployed to the RoboRio that have been deleted but keep populating Elastic, you can open WinSCP and connect to the RoboRio and delete the auto files.
    - A backup of the WinSCP connection can be found in this repo: [WinSCP.ini](WinSCP.ini)

## Adding New Components

- Create/update motors or other components in [robot/components/](./src/main/java/frc/robot/components/).
  - Keeping all components here allows for easier upgrades year-over-year. If you change something here, it will be applied to all places the component is used in the subsystems and commands.
- When adding components, define an Interface for functions (such as [Motor.java](./src/main/java/frc/robot/components/motors/lib/Motor.java)) that all component types can do and a config of all inputs needed to configure the component in the component's lib directory (such as [MotorConfig.java](./src/main/java/frc/robot/components/motors/lib/MotorConfig.java)).
- Add one or more named components in the component's directory (such as [Kraken.java](./src/main/java/frc/robot/components/motors/Kraken.java)) that implement the Interfaces in the lib directory.
