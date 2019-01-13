package example;

import simbad.sim.*;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

public class Robot extends Agent {
	
	// 全局目标坐标
    Vector2d goal = new Vector2d(8,8 );
    Vector3d goal3d = new Vector3d(8, 0,8);

    private static final double repelConstant = 200.0;// 斥力系数
    private static final double attractConstant = 30.0;// 引力系数
    //初始位置
    private Vector3d origin = null;
    //声呐、保险扛和信号灯
    RangeSensorBelt sonars,bumpers;
    LampActuator lamp;

    public Robot(Vector3d position, String name) {
        super(position, name);
        //ʹ��RobotFactory������ȡRangeSensorBelt���� -> ���� 这货是乱码，那我是什么
        sonars = RobotFactory.addSonarBeltSensor(this,8);
        bumpers = RobotFactory.addBumperBeltSensor(this,8);
        lamp = RobotFactory.addLamp(this);
        origin = position;// 起点位置
    }
    
    //线速度
    public Vector3d getVelocity()
    {
        return this.linearVelocity;
    }
    
  //计算斥力
    private double repelForce(double distance, double range) 
    {
        double force = 0;
        Point3d p = new Point3d();
        getCoords(p); //获取当前坐标
        
        return force;
    }
    
  //计算吸引力
    private double attractForce(double distance) 
    {
        double force = attractConstant * distance;
        return force;
    }
    
  //检查是否到达目的地
    private boolean checkGoal() 
    {

        Point3d currentPos = new Point3d();
        getCoords(currentPos); //当前坐标
        Point3d goalPos = new Point3d(goal3d.x, goal3d.y, goal3d.z);

        if (currentPos.distance(goalPos) <= 0.5) // 如果当前距离目标点小于0.5那么即认为是到达
        {
            return true;
        } else
        {
            return false;
        }
    }

   public void performBehavior() {
	   
       //every 20 frames
       if (getCounter()%20==0){
           // print each sonars measurement ��ӡ��ÿһ�����ɵĲ���ֵ	
           for (int i=0;i< sonars.getNumSensors();i++) {
               double range = sonars.getMeasurement(i); 
               boolean hit = sonars.hasHit(i);
               double angle = bumpers.getSensorAngle(i);
               System.out.println("measured range ="+range+ " has hit something:"+hit); 
           }
           
           if (collisionDetected()) {
               // stop the robot
               setTranslationalVelocity(0.0);
               setRotationalVelocity(0);
           } else {
               // progress at 0.5 m/s 
               setTranslationalVelocity(0.5);
               // frequently change orientation 
               // getCounter
               if ((getCounter() % 100)==0) 
               	
                  setRotationalVelocity(Math.PI/2 * (0.5 - Math.random()));
           }
       }
   }

}
