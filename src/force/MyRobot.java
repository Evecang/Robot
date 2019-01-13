package force;

import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import simbad.sim.*;
import simbad.demo.*;
import simbad.gui.Simbad;

public class MyRobot extends Demo
{
    // 全局目标坐标
    Vector2d goal = new Vector2d(8,8);
    Vector3d goal3d = new Vector3d(8,0,8);

    private static final double repelConstant = 200.0;// 斥力系数
    private static final double attractConstant = 30.0;// 引力系数
    private boolean debug = true;// 调试标记

    public class Robot extends Agent
    {

        RangeSensorBelt sonars,bumpers;
        LampActuator lamp;

        private Vector3d origin = null;

        public void initBehavior()
        {
        }

        public Robot(Vector3d position, String name)
        {
            super(position, name);

            bumpers = RobotFactory.addBumperBeltSensor(this);
            sonars = RobotFactory.addSonarBeltSensor(this);
            lamp = RobotFactory.addLamp(this);
            origin = position;// 起点位置

        }

        public Vector3d getVelocity()
        {
            return this.linearVelocity; //线速度
        }

        private int getQuadrant(Vector2d vector) //计算向量的象限
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
            } else//原点
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

        private double getAngle(Vector2d v1, Vector2d v2) //计算两个向量之间的弧度角
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

        private double repelForce(double distance, double range) //计算斥力
        {
            double force = 0;
            if (distance <= range) //距离大于range则没有斥力
            {
//                force = (range-distance)*repelConstant;//计算斥力
            	force = repelConstant/distance;
            }

            return force;
        }

        private double attractForce(double distance) //计算吸引力
        {
            double force = attractConstant * distance;
            return force;
        }

        private boolean checkGoal() //检查是否到达目的地
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


        public void performBehavior()
        {
            // 为了防止智能体剧烈晃动，每10帧计算一次受力
            if (getCounter() % 10 == 0)
            {

                Vector3d velocity = getVelocity(); //获取速度


                Vector2d direct = new Vector2d(velocity.z, velocity.x); //前进的方向向量

                //获得机器人的位置
                Point3d p = new Point3d();
                getCoords(p);
                Vector2d pos = new Vector2d(p.z, p.x);
                
                //计算斥力
                int sonarNum = sonars.getNumSensors();	//声纳个数
                double fd[] = new double[sonarNum];		//各个声纳检测到的距离
                for(int i=0;i<sonarNum;i++){
//                	vf[i] = repelForce(sonars.getMeasurement(i),2.0);
                	fd[i] = sonars.getMeasurement(i);
                }
                //
                double vfx = 0.0;
                double vfy = 0.0;
                double angleToX = getAngle(new Vector2d(1,0), direct);	//运动方向到x轴的角度
                
                for(int i=0;i<sonarNum;i++){
                	if(fd[i] <= 2.0){
	                	vfx += -repelForce(fd[i],2.0) * ( Math.cos(sonars.getSensorAngle(i) + angleToX) );
	                	vfy += repelForce(fd[i],2.0) * ( Math.sin(sonars.getSensorAngle(i) + angleToX) );
                	}
                }
                
                Vector2d goalRepelForce = new Vector2d(vfx, vfy);
                
                if (debug)
                    System.out.println("partRepelForce(" + goalRepelForce.x + ","
                            + goalRepelForce.y);
                

//                //得到前面三个声纳的测量值
//                double d0 = sonars.getMeasurement(0);// front声纳，正前方
//                double d1 = sonars.getMeasurement(1);// frontleft声纳，左前方
//                double d2 = sonars.getMeasurement(8);// frontright声纳，右前方
//
//                //使用三次测量计算三个排斥力
//                double rf0 = repelForce(d0,2.0); //三个方向的斥力
//                double rf1 = repelForce(d1, 2.0);
//                double rf2 = repelForce(d2, 2.0);
//                System.out.println("d0="+d0 + "    D1="+d1 +"    d2="+d2);
//                System.out.println("rf0="+rf0 + "    rf1="+rf1 +"    rf2="+rf2);
//
//                // 计算斥力的合力
//                //计算局部参考系中三个排斥力的组成
//                double k1 = Math.cos(2 * Math.PI / 9);	//40
//                double k2 = Math.sin(2 * Math.PI / 9);	//40
//                Vector2d vf0 = new Vector2d(0 - rf0, 0);
//                Vector2d vf1 = new Vector2d((0 - rf1 * k1), (0 - rf1 * k2));
//                Vector2d vf2 = new Vector2d((0-rf2 * k1), (0-rf2 * k2));
//                Vector2d composition = new Vector2d();
//                System.out.println("vf0.x:"+vf0.x+",vf1.x:"+vf1.x+",vf2.x:"+vf2.x);
//                System.out.println("vf1.y:"+vf0.y+",vf1.y:"+vf1.y+",vf2.y"+vf2.y);
//                composition.setX(vf0.x + vf1.x + vf2.x);
//                composition.setY(vf0.y + vf1.y + vf2.y);

//                if (debug)
//                    System.out.println("(" + composition.x + ","
//                            + composition.y);

                //将排斥力的组成转换为全局坐标
                // Vector2d repelForceVector = transform(direct, composition);


                //计算目标的吸引力
                Vector2d toGoal = new Vector2d((goal.x - pos.x),
                        (goal.y - pos.y));
                double disGoal = toGoal.length();
                if (debug)
                    System.out.println("distance to goal:" + disGoal);
                double goalForce = attractForce(disGoal);

                if (debug)
                    System.out.println("attract force from goal:" + goalForce);
                
                Vector2d goalAttractForce = new Vector2d(
                        (goalForce * toGoal.x / disGoal),
                        (goalForce * toGoal.y / disGoal));
                
//                Vector2d originForceVector = new Vector2d(origin.x, origin.z);

                //斥力与吸引力的合力
                double x = goalRepelForce.x + goalAttractForce.x;
                double y = goalRepelForce.y + goalAttractForce.y;

                Vector2d allForces = new Vector2d(x, y);
                
                if (debug)
                {
                    System.out.println("total force(" + allForces.x + ","
                            + allForces.y + ")");
                    System.out.println("force direct(" + direct.x + ","
                            + direct.y + ")");
                }
                
                //TODO:如果合力为0的情况：
                

                //根据力决定机器人应该移动的方向，direct为运动速度的方向向量
                double angle = getAngle(direct, allForces);


                if (debug)
                    System.out.println("angle:" + angle);

//                // 判断转动方向
//                if (angle < Math.PI)
//                {
//                    setRotationalVelocity(angle);
//                } else if (angle > Math.PI)
//                {
//                    setRotationalVelocity((angle - 2 * Math.PI));
//                }
                //让机器人调整运动方向
                setRotationalVelocity(angle);

                if (checkGoal())
                {
                    // 到达目标点，停止运动
                    setTranslationalVelocity(0);
                    setRotationalVelocity(0);
                    lamp.setOn(true);
                    return;
                } else
                {
                    lamp.setOn(false);
                    setTranslationalVelocity(0.5);
                }

                // 检测是否碰撞
                if (bumpers.oneHasHit())
                {
                    lamp.setBlink(true);

                    double left = sonars.getFrontLeftQuadrantMeasurement();
                    double right = sonars.getFrontRightQuadrantMeasurement();
                    double front = sonars.getFrontQuadrantMeasurement();

                    if ((front < 0.7) || (left < 0.7) || (right < 0.7))
                    {
                        if (left < right)
                        {
                            setRotationalVelocity(-1 - (0.1 * Math.random()));// 随机向右转 -1.。。
                        } else
                        {
                            setRotationalVelocity(1 - (0.1 * Math.random()));// 随机向左转	+0.。。
                        }
                        setTranslationalVelocity(0);
                    }
                }
                else lamp.setBlink(false);
            }
        }
    }

    static public class MyMovRobot extends Agent{

        RangeSensorBelt sonars,bumpers;
        LampActuator lamp;
        double speed = 0.5;

        public MyMovRobot (Vector3d position, String name) {
            super(position,name);
            bumpers = RobotFactory.addBumperBeltSensor(this);
            sonars = RobotFactory.addSonarBeltSensor(this,24);
            lamp = RobotFactory.addLamp(this);
        }

        public void initBehavior() {
            setTranslationalVelocity(speed);
        }

        public void performBehavior() {

            if (bumpers.oneHasHit()) {
                lamp.setBlink(true);
            }else
                lamp.setBlink(false);

            if(getCounter()%70 == 0){
                speed = -speed;
                setTranslationalVelocity(speed);
            }
        }
    }

    public MyRobot()
    {
    	//环境的边界
        Wall w1 = new Wall(new Vector3d(10, 0, 0), 20, 1, this);
        w1.rotate90(1);
        add(w1);
        Wall w2 = new Wall(new Vector3d(-10, 0, 0), 20, 1, this);
        w2.rotate90(1);
        add(w2);
        Wall w3 = new Wall(new Vector3d(0, 0, 10), 20, 1, this);
        add(w3);
        Wall w4 = new Wall(new Vector3d(0, 0, -10), 20, 1, this);
        add(w4);


        //墙壁障碍物
        Wall w5 = new Wall(new Vector3d(6, 0, 0), 3, 1, this);
        add(w5);


        Wall w9 = new Wall(new Vector3d(-7, 0, -7), 1, 1, this);
        w9.rotate90(1);
        add(w9);
        Wall w10 = new Wall(new Vector3d(5, 0, 7),1, 1, this);
        w10.rotate90(1);
        add(w10);

        Box b1 = new Box(new Vector3d(0,0,0), new Vector3f(3, 3, 3),this);
        add(b1);
        Box b2 = new Box(new Vector3d(-4,0,-4), new Vector3f(2, 1, 3),this);
        add(b2);

        Arch a1=new Arch(new Vector3d(0,0,6),this);
        a1.rotate90(2);
        add(a1);

        add(new MyRobot.Robot(new Vector3d(-8, 0, -8), "My Robot"));
        add(new MyMovRobot(new Vector3d(5, 0, 3), "MyMovRObot"));

    }

    public static void main(String[] args)
    {
        //System.setProperty("j3d.implicitAntialiasing", "true");
        Simbad frame = new Simbad(new MyRobot(), false);
    }
}

