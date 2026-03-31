// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.cameras;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import frc.robot.components.cameras.lib.Camera;
import frc.robot.components.cameras.lib.CameraConfig;

public class USBCamera implements Camera {

    private final CameraConfig config;
    private final UsbCamera camera;

    public USBCamera(CameraConfig cameraConfig) {

        this.config = cameraConfig;
        this.camera = CameraServer.startAutomaticCapture(config.name, config.portId);

        configureCamera();
    }

    private void configureCamera() {
        camera.setResolution(config.width, config.height);
        camera.setFPS(config.framesPerSecond);

        if (config.exposure == -1) {
            camera.setExposureAuto();
        }
        else {
            camera.setExposureManual(config.exposure);
        }
    }

    @Override
    public String getName() {
        return config.name;
    }

    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    @Override
    public void setFPS(int fps) {
        camera.setFPS(fps);
    }

    @Override
    public void setResolution(int width, int height) {
        camera.setResolution(width, height);
    }

    @Override
    public VideoSource getVideoSource() {
        return camera;
    }
}
