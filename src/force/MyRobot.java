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
    // ȫ��Ŀ������:3d��z = 2d��-y   3d��x = 2d��x
    Vector2d goal = new Vector2d(8,-8);
    Vector3d goal3d = new Vector3d(8,0,8);

    //216��������/13(����) -������Ҫ�����С ���������500һ��
    private static final double repelConstant = 216.0;// ����ϵ��
    private static final double attractConstant = 400.0;// ����ϵ��
    private boolean debug = true;// ���Ա��	

    public class Robot extends Agent
    {

        RangeSensorBelt sonars,bumpers;
        LampActuator lamp;

        private Vector3d origin = null;
        
        long startTime;   //��ȡ��ʼʱ��

        public void initBehavior()
        {
        	
        }

        public Robot(Vector3d position, String name)
        {
            super(position, name);

            bumpers = RobotFactory.addBumperBeltSensor(this);
            sonars = RobotFactory.addSonarBeltSensor(this);
            lamp = RobotFactory.addLamp(this);
            origin = position;// ���λ��

        }

        public Vector3d getVelocity()
        {
            return this.linearVelocity; //���ٶ�
        }

        private int getQuadrant(Vector2d vector) //��������������
        {
            double x = vector.x;
            double y = vector.y;
            if (x > 0 && y > 0)// ��һ����
            {
                return 1;
            } else if (x < 0 && y > 0)// �ڶ�����
            {
                return 2;
            } else if (x <0 && y < 0)// ��������
            {
                return 3;
            } else if (x > 0 && y <0)// ��������
            {
                return 4;
            } else if (x > 0 && y == 0)// x������
            {
                return -1;
            } else if (x == 0 && y > 0)// y������
            {
                return -2;
            } else if (x <0 && y == 0)// x������
            {
                return -3;
            } else if (x == 0 && y <0)// y������
            {
                return -4;
            } else//ԭ��
            {
                return 0;
            }
        }
        
    	private double getToXAngle(Vector2d v)	//����������x��ĽǶ�
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
	        	   System.out.println("getAngle->getToXAngle�����������ԭ��");
	        	   return v.angle(new Vector2d(1,0));
            }
            
    	}

        private double getAngle(Vector2d v1, Vector2d v2) //������������֮��Ļ��Ƚ�
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

        private double repelForce(double distance, double range, double toGoal) //�������
        {
            double force = 0;
            if (distance <= range) //�������range��û�г���
            {
                //�������
//            	force = repelConstant/(distance*distance);
            	force = 0.5 * repelConstant * Math.pow((1/distance - 1/range), 2) * Math.pow(toGoal,2);
            }

            return force;
        }

        private double attractForce(double distance) //����������
        {
//            double force = attractConstant * distance;
        	double force = 0.5 * attractConstant * Math.pow(distance,2);
            return force;
        }

        private boolean checkGoal() //����Ƿ񵽴�Ŀ�ĵ�
        {

            Point3d currentPos = new Point3d();
            getCoords(currentPos); //��ǰ����
            Point3d goalPos = new Point3d(goal3d.x, goal3d.y, goal3d.z);

            if (currentPos.distance(goalPos) <= 0.5) // �����ǰ����Ŀ���С��0.5��ô����Ϊ�ǵ���
            {
                return true;
            } else
            {
                return false;
            }
        }


        public void performBehavior()
        {
        	if(getCounter() == 0){
        		startTime = System.currentTimeMillis();   //��ȡ��ʼʱ��
        	}
            // Ϊ�˷�ֹ��������һζ���ÿ10֡����һ������
            if (getCounter() % 10 == 0)
            {

                Vector3d velocity = getVelocity(); //��ȡ�ٶ�


                Vector2d direct = new Vector2d(velocity.x, -velocity.z); //ǰ���ķ�������

                //��û����˵�λ��
                Point3d p = new Point3d();
                getCoords(p);
                Vector2d pos = new Vector2d(p.x, -p.z);
                
                //�������
                int sonarNum = sonars.getNumSensors();	//���ɸ���
                double fd[] = new double[sonarNum];		//�������ɼ�⵽�ľ���
                for(int i=0;i<sonarNum;i++){
                	fd[i] = sonars.getMeasurement(i);
                }
                double vfx = 0.0;
                double vfy = 0.0;
                double angleToX = getAngle(new Vector2d(1,0), direct);	//�˶�����x��ĽǶ�
                

                //��ǰλ�õ�Ŀ���ľ���
                Vector2d toGoal = new Vector2d((goal.x - pos.x),
                        (goal.y - pos.y));
                double disGoal = toGoal.length(); 
                
                if (debug)
                    System.out.println("distance to goal:" + disGoal);

                
                
                for(int i=0;i<sonarNum;i++){
                	if(fd[i] <= 2.0 ){
	                	vfx += -repelForce(fd[i],2.0,disGoal) * ( Math.cos(sonars.getSensorAngle(i) + angleToX) );
	                	vfy += -repelForce(fd[i],2.0,disGoal) * ( Math.sin(sonars.getSensorAngle(i) + angleToX) );
                	}
                }
                
                Vector2d goalRepelForce = new Vector2d(vfx, vfy);
                
                if (debug)
                    System.out.println("ȫ��2d����(" + goalRepelForce.x + ","
                            + goalRepelForce.y + ")");

                //����Ŀ���������
                double goalForce = attractForce(disGoal);
                Vector2d goalAttractForce = new Vector2d(
                        (goalForce * toGoal.x / disGoal),
                        (goalForce * toGoal.y / disGoal));
                
                if (debug)
                    System.out.println("ȫ��2d����:" + goalAttractForce);
                
                //�������������ĺ���
                double x = goalRepelForce.x + goalAttractForce.x;
                double y = goalRepelForce.y + goalAttractForce.y;

                Vector2d allForces = new Vector2d(x, y);
                
                if (debug)
                {
                    System.out.println("����2d(" + allForces.x + ","
                            + allForces.y + ")");
                    System.out.println("�˶�����(" + direct.x + ","
                            + direct.y + ")");
                }
                
                //TODO:�������Ϊ0�������
                

                //����������������Ӧ���ƶ��ķ���directΪ�˶��ٶȵķ�������
                double angle = getAngle(direct, allForces);

                if (debug)
                    System.out.println("��ת����:" + angle / Math.PI * 180);


                //�û����˵����˶�����
                setRotationalVelocity(angle);

                if (checkGoal())
                {
                    // ����Ŀ��㣬ֹͣ�˶�
                    setTranslationalVelocity(0);
                    setRotationalVelocity(0);
                    lamp.setOn(true);
                    long endTime=System.currentTimeMillis(); //��ȡ����ʱ��
//                    debug = false;
                    System.out.println("��������ʱ�䣺 "+(endTime-startTime)+"ms");
                    return;
                } else
                {
                    lamp.setOn(false);
                    setTranslationalVelocity(0.5);
                }

                // ����Ƿ���ײ
                if (bumpers.oneHasHit())
                {
                    lamp.setBlink(true);

//                    double left = sonars.getFrontLeftQuadrantMeasurement();
//                    double right = sonars.getFrontRightQuadrantMeasurement();
//                    double front = sonars.getFrontQuadrantMeasurement();
//
//                    if ((front < 0.7) || (left < 0.7) || (right < 0.7))
//                    {
//                        if (left < right)
//                        {
//                            setRotationalVelocity(-1 - (0.1 * Math.random()));// ����ת
//                        } else
//                        {
//                            setRotationalVelocity(1 - (0.1 * Math.random()));// ����ת
//                        }
//                        setTranslationalVelocity(0);
//                    }
                    
//                    setTranslationalVelocity(0);
                    setRotationalVelocity(angle);
                    setTranslationalVelocity(0.5);
                    
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
    	//�����ı߽�
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


        //ǽ���ϰ���
        Wall w5 = new Wall(new Vector3d(6, 0, 0), 3, 1, this);
        add(w5);

        Wall w6 = new Wall(new Vector3d(3, 0, -6), 4, 1, this);
//        w6.rotate90(1);
        add(w6);
        
//        Wall w7 = new Wall(new Vector3d(-9, 0, -7), 4, 1, this);
////        w6.rotate90(1);
//        add(w7);

        Wall w9 = new Wall(new Vector3d(-7, 0, -6), 2, 1, this);
        w9.rotate90(1);
        add(w9);
        
        Wall w10 = new Wall(new Vector3d(5, 0, 7),1, 1, this);
        w10.rotate90(1);
        add(w10);

        Box b1 = new Box(new Vector3d(0,0,0), new Vector3f(3, 3, 3),this);
        add(b1);
//        Box b2 = new Box(new Vector3d(-4,0,-4), new Vector3f(2, 1, 3),this);
//        add(b2);
        
        Arch a2=new Arch(new Vector3d(-5,0,-4),this);
//        a1.rotate90(2);
        add(a2);

        Arch a1=new Arch(new Vector3d(0,0,6),this);
        a1.rotate90(2);
        add(a1);
        Arch a3=new Arch(new Vector3d(-5,0,6),this);
        a3.rotate90(2);
        add(a3);

        add(new MyRobot.Robot(new Vector3d(-8, 0, -8), "My Robot"));
        add(new MyMovRobot(new Vector3d(5, 0, 3), "MyMovRObot1"));
        add(new MyMovRobot(new Vector3d(-6, 0, 0), "MyMovRObot2"));
        add(new MyMovRobot(new Vector3d(0, 0, -4), "MyMovRObot3"));

    }

    public static void main(String[] args)
    {
        //System.setProperty("j3d.implicitAntialiasing", "true");
        Simbad frame = new Simbad(new MyRobot(), false);
    }
}

