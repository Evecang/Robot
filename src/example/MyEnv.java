package example;

import simbad.sim.*;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import simbad.sim.Wall;
import simbad.sim.Box;

public class MyEnv extends EnvironmentDescription{
    public MyEnv(){
    	//Arch预建对象之一，如Wall或Box
    	//MyRobot自己提供的机器人对象，使用MyRobot类描述您的机器人控制器，MyRobot派生自Agent类
        add(new Arch(new Vector3d(3,0,-3),this));
//        add(new Arch(new Vector3d(-5,0,-3),this));
//        add(new Arch(new Vector3d(5,0,3),this));
        add(new Wall(new Vector3d(-5,0,3),1,1,this));
        add(new Box(new Vector3d(1,0,1),new Vector3f(1,3,1),this));
        
//        Box b2 = new Box(new Vector3d(-4,0,-4), new Vector3f(1, 3, 1),this);   
//        add(b2);
        
        add(new Robot(new Vector3d(0, 0, 0),"my robot"));
    }
}
