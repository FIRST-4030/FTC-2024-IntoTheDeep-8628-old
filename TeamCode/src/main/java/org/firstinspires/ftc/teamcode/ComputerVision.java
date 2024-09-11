package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class ComputerVision {

    public AprilTagProcessor aprilTagProcessor;
    //AprilTagProcessor.Builder aprilTagBuilder;
    public VisionPortal visionPortal;
    //VisionPortal.Builder visionPortalBuilder;
    String[] labels = {"Blue Prop", "Red Prop"};

    public ArrayList<AprilTagDetection> aprilTagDetections;
    public int spike = 1;
    Pose2d robotPose = new Pose2d(0, 0, 0);
    WebcamName webcam1, webcam2;


    public static final ArrayList<Pose2d> aprilTagPoses = new ArrayList<>(Arrays.asList(
            new Pose2d(0, 0, 0), //buffer to stop IDs from being zero-index
            new Pose2d(62, 41.4, 0), //1
            new Pose2d(62, 35.5, 0), //2
            new Pose2d(62, 29.3, 0), //3
            new Pose2d(62, -29.3, 0), //4
            new Pose2d(62, -35.5, 0), //5
            new Pose2d(62, -41.4, 0), //6
            new Pose2d(-70.6, -41, 0), //7
            new Pose2d(-70.6, -35.5, 0), //8
            new Pose2d(-70.6, 35.5, 0), //9
            new Pose2d(-70.6, 41, 0)  //10
    ));

    AprilTagPoseFtc currentTagTranslation;
    AprilTagPoseFtc[] aprilTagTranslations = new AprilTagPoseFtc[11];

    public ComputerVision(HardwareMap hardwareMap){

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                //.setLensIntrinsics(952.837, 952.837, 622.758, 398.223)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessors(aprilTagProcessor)
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();
    }

    public void updateAprilTags(){
        aprilTagDetections = aprilTagProcessor.getDetections();
    }

    public AprilTagPoseFtc[] getTranslationToTags(){
            for(int i=0;i<aprilTagTranslations.length;i++)
            {
                aprilTagTranslations[i] = null;
            }
            aprilTagDetections.forEach((AprilTagDetection) -> {
                aprilTagTranslations[AprilTagDetection.id] = AprilTagDetection.ftcPose;
            });
            return aprilTagTranslations;
        }
        //returns the absolute robot position based on the april tag pose, requires the id of the april tag as well as whether to use the front camera or not
        public Pose2d localize(int id, boolean frontCam){
            try {
                currentTagTranslation = getTranslationToTags()[id];
                Pose2d aprilTagPose = aprilTagPoses.get(id);
                robotPose = new Pose2d(new Vector2d(
                        frontCam ? aprilTagPose.position.x - currentTagTranslation.y - 8 : aprilTagPose.position.x + currentTagTranslation.y + 8,
                        frontCam ? aprilTagPose.position.y + currentTagTranslation.x : aprilTagPose.position.y - currentTagTranslation.x),
                0);
                return robotPose;
            } catch(Exception e) {
                    robotPose = new Pose2d(0,0,0);
                    return robotPose;
            }
        }

        public ArrayList<AprilTagDetection> getAprilTagDetections () {
            return aprilTagDetections;
        }
        public void setActiveCameraOne() {
            visionPortal.setActiveCamera(webcam1);
        }
    public void setActiveCameraTwo() {
        visionPortal.setActiveCamera(webcam2);
    }
    }