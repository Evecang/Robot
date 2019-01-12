package example;

import simbad.sim.*;
import javax.vecmath.Vector3d;

public class MyRobot extends Agent {
    public MyRobot (Vector3d position, String name) {     
        super(position,name);
    }
    public void initBehavior() {}
    
    public void performBehavior() {
    	//collisionDetected 如果碰撞了就返回true
        if (collisionDetected()) {
            // stop the robot
        	//设置以米/秒为单位的平移速度
            setTranslationalVelocity(0.0);
            //以每秒弧度为单位设置旋转速度
            setRotationalVelocity(0);
        } else {
            // progress at 0.5 m/s -> 以0.5米/秒的速度前进
            setTranslationalVelocity(0.5);
            // frequently change orientation 
            // getCounter->经过的模拟步骤数,
            if ((getCounter() % 100)==0) 
            	//以每秒弧度为单位设置旋转速度
               setRotationalVelocity(Math.PI/2 * (0.5 - Math.random()));
        }
    }
}
