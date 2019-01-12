package example;

import simbad.sim.*;
import javax.vecmath.Vector3d;

public class Robot extends Agent {
    RangeSensorBelt sonars;
    RangeSensorBelt bumpers;

    public Robot(Vector3d position, String name) {
        super(position, name);
        //ʹ��RobotFactory������ȡRangeSensorBelt���� -> ����
        sonars = RobotFactory.addSonarBeltSensor(this,8);
        bumpers = RobotFactory.addBumperBeltSensor(this,8);
    }

   public void performBehavior() {
	   if (collisionDetected()) {
           // stop the robot
       	//��������/��Ϊ��λ��ƽ���ٶ�
           setTranslationalVelocity(0.0);
           //��ÿ�뻡��Ϊ��λ������ת�ٶ�
           setRotationalVelocity(0);
       } else {
           // progress at 0.5 m/s -> ��0.5��/����ٶ�ǰ��
           setTranslationalVelocity(0.5);
           // frequently change orientation 
           // getCounter->������ģ�ⲽ����,
           if ((getCounter() % 100)==0) 
           	//��ÿ�뻡��Ϊ��λ������ת�ٶ�
              setRotationalVelocity(Math.PI/2 * (0.5 - Math.random()));
       }
       //every 20 frames
       if (getCounter()%20==0){
           // print each sonars measurement ��ӡ��ÿһ�����ɵĲ���ֵ	
           for (int i=0;i< sonars.getNumSensors();i++) {
               double range = sonars.getMeasurement(i); 
               boolean hit = sonars.hasHit(i);
               double angle = bumpers.getSensorAngle(i);
               System.out.println("measured range ="+range+ " has hit something:"+hit); 
           }
       }
   }
	
//    CameraSensor camera;
////    BufferedImage cameraImage;
//
//    public Robot(Vector3d position, String name) {
//        super(position, name);
//        // add a camera on top of the robot
//        camera = RobotFactory.addCameraSensor(this);
//        // reserve space for image capture
////        cameraImage = camera.createCompatibleImage();
//   }
//
//     public void performBehavior() {
////          .......  your code
//
//         // get camera image 
//         camera.copyVisionImage(cameraImage);
//         // process image 
////         ... use BufferedImage api
//     } 
}
