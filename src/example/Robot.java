package example;

import simbad.sim.*;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

public class Robot extends Agent {
	
	// 全局目标坐标
    Vector2d goal = new Vector2d(8,8);
    Vector3d goal3d = new Vector3d(8, 0,8);

    private static final double repelConstant = 200.0;// 斥力系数
    private static final double attractConstant = 30.0;// 引力系数
    //初始位置
    private Vector3d origin = null;
    //声呐、保险扛和信号灯
    RangeSensorBelt sonars,bumpers;
    LampActuator lamp;
    
    double force[] = new double[8];

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
    
  //计算向量的象限
    private int getQuadrant(Vector2d vector) 
    {
        double x = vector.x;
        double y = vector.y;
        if (x > 0 && y > 0)// 第一象限
        {
            return 1;
        } else if (x < 0 && y > 0)// 第二象限
        {
            return 2;
        } else if (x <0 && y < 0)// 第三象限
        {
            return 3;
        } else if (x > 0 && y <0)// 第四象限
        {
            return 4;
        } else if (x > 0 && y == 0)// x正半轴
        {
            return -1;
        } else if (x == 0 && y > 0)// y正半轴
        {
            return -2;
        } else if (x <0 && y == 0)// x负半轴
        {
            return -3;
        } else if (x == 0 && y <0)// y负半轴
        {
            return -4;
        } else
        {
            return 0;
        }
    }

    
    private double getToXAngle(Vector2d v)	//计算向量到x轴的角度
	{
	
        switch(getQuadrant(v)){
        case 1:
        case 2:
        case -1:
        case -2:
        case -3:
        	return v.angle(new Vector2d(1,0));
        case 3:
        case 4:
        case -4:
        	return 2 * Math.PI - v.angle(new Vector2d(1,0));
       default:
    	   System.out.println("getAngle->getToXAngle输入的向量是原点");
    	   return v.angle(new Vector2d(1,0));
        }
        
	}

    //计算两个向量之间的弧度角
    private double getAngle(Vector2d v1, Vector2d v2) 
    {
    	double angle1 = getToXAngle(v1);
    	double angle2 = getToXAngle(v2);
    	if(angle1 > angle2){
	    	if(angle1 > (angle2 + Math.PI)){
	    		return angle2 - angle1 + 2 * Math.PI;
	    	}else{
	    		return -(angle1 - angle2);
	    	}
    	}else if(angle1 < angle2){
	    	if(angle2 > (angle1 + Math.PI)){
	    		return -(2 * Math.PI - angle2 + angle1);
	    	}else{
	    		return angle2 - angle1;
	    	}
    	}else{
    		return 0.0;
    	}
    }
    
  //计算斥力 TODO
    private double repelForce(double distance, double range) 
    {
        double force = 0;
        Point3d p = new Point3d();
        getCoords(p); //获取当前坐标
        //TODO
        return force;
    }
    
    //计算斥力的合力
    private Vector2d getRepelVector(double force[],Vector2d direction) {
    	double x = 0.0, y=0.0;
    	double angle = getAngle(new Vector2d(1,0), direction);
    	for(int i = 0; i < force.length; i++) {
    	if(force[i] != 0) {
    	x += repelConstant / force[i] * (-Math.cos(sonars.getSensorAngle(i) + angle));
    	y += repelConstant / force[i] * Math.sin(sonars.getSensorAngle(i) + angle);
    	}
    	}
    	Vector2d repel = new Vector2d(x,y);
    	return repel;
    }
    
  //计算吸引力TODO
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
           // print each sonars measurement 
           for (int i=0;i< sonars.getNumSensors();i++) {
               double distance = sonars.getMeasurement(i); 
               boolean hit = sonars.hasHit(i);
               double angle = bumpers.getSensorAngle(i);
               System.out.println("measured range ="+distance+ " has hit something:"+hit); 
           }
           
           if (collisionDetected()) {
               // stop the robot
               setTranslationalVelocity(0);
               setRotationalVelocity(Math.PI * Math.random());
               setTranslationalVelocity(0.3);
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
