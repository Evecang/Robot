package example;

import simbad.sim.*;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import simbad.sim.Wall;
import simbad.sim.Box;

public class MyEnv extends EnvironmentDescription{
    public MyEnv(){
    	//Build enviroment: 4 Wall
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
    	//ArchԤ������֮һ����Wall��Box
    	//MyRobot�Լ��ṩ�Ļ����˶���ʹ��MyRobot���������Ļ����˿�������MyRobot������Agent��
        add(new Arch(new Vector3d(3,0,-3),this));
//        add(new Arch(new Vector3d(-5,0,-3),this));
//        add(new Arch(new Vector3d(5,0,3),this));
        add(new Wall(new Vector3d(-5,0,3),1,1,this));
        add(new Box(new Vector3d(1,0,1),new Vector3f(1,3,1),this));
        
//        Box b2 = new Box(new Vector3d(-4,0,-4), new Vector3f(1, 3, 1),this);   
//        add(b2);
        
        add(new Robot(new Vector3d(-8, 0, -8),"my robot"));
    }
}
