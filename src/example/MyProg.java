package example;

import javax.vecmath.Vector2d;

import simbad.gui.Simbad;

public class MyProg {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		//EnvironmentDescription.class -> MyEnv
//		Simbad frame = new Simbad(new MyEnv() ,false);
		Vector2d v = new Vector2d(1,1);
		System.out.println(getToXAngle(v));
	}
	
	private static double getToXAngle(Vector2d v)	//����������x��ĽǶ�
	{
		int a = 1;
        switch(a){
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

}
