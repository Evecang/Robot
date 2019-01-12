package example;

import simbad.sim.*;
import javax.vecmath.Vector3d;

public class MyRobot extends Agent {
    public MyRobot (Vector3d position, String name) {     
        super(position,name);
    }
    public void initBehavior() {}
    
    public void performBehavior() {
    	//collisionDetected �����ײ�˾ͷ���true
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
    }
}
