// bidirectional connection between client and server 
// secondary thread to the main file "TestingServer" 

package application;

import java.net.*;
import java.io.*;
import java.lang.Thread;
import com.kuka.common.ThreadUtil;   
import com.kuka.generated.ioAccess.RobotiqGrip3IOGroup;
import com.kuka.roboticsAPI.deviceModel.LBR;

import com.kuka.roboticsAPI.sensorModel.ForceSensorData;     
import com.kuka.roboticsAPI.sensorModel.SensorForMeasuredTorque;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Tool;
//Default
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication; // <--
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;       // <--
import com.kuka.roboticsAPI.controllerModel.Controller;              // <--
import com.kuka.roboticsAPI.conditionModel.ForceComponentCondition;
import com.kuka.roboticsAPI.conditionModel.TorqueComponentCondition;

import com.kuka.roboticsAPI.geometricModel.CartDOF;                  // <--
import com.kuka.roboticsAPI.geometricModel.Frame;                       // <--

import com.kuka.roboticsAPI.geometricModel.World;                    // <--
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;      
import com.kuka.roboticsAPI.motionModel.IDirectServoRuntime;       // <--

import com.kuka.roboticsAPI.motionModel.SmartServo; 
import com.kuka.roboticsAPI.motionModel.DirectServo;               // <--
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode; // <--         // <--
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;   // <--


public class YW_UDP_Server_v3_communicator extends Thread {
	protected DatagramSocket socket = null;
	protected String recvStr = new String();
	protected Boolean end = false;
	protected Boolean close = false;
	protected long LastAccess = System.currentTimeMillis();
	byte[] send_data = new byte[1024];
	protected DatagramPacket recvPacket;
	
	public boolean hasReceived = false;

	private Tool ToolGripper;
	private RobotiqGrip3IOGroup FingerTip;
	
	protected double[][] data_pos;
	protected double[][] data_torque;
	protected double[][] data_ext;
	protected double[] data_time;
	protected int sample_f;
	protected long start_nanotime;
	protected int sample_n;
	private LBR lbr_iiwa_14_R820_1;
	private int max_record_steps=0;
	public YW_UDPThread client;
	public int communicator_loop=0;
	
	public int ControlMode;
	public double[] xyzabgDouble_sp=new double[7]; 
	public double[] TCPxyzabgDouble_sp=new double[6]; 
	public double[] JointsDouble_sp=new double[7];
	public double[] ForceDouble_sp=new double[6];
	public double SpeedDouble_sp;
	public double[] stiff_sp=new double[6];
	public double[] JointsSpeed_sp=new double[7];
	public double blending_sp;
	public double[] MaxForce_sp=new double[6];
	public double[] jointAngle = new double[8];
	
	public void init(LBR iiwa,YW_UDPThread client_in,RobotiqGrip3IOGroup FingerTip_in,Tool ToolGripper_in){
		lbr_iiwa_14_R820_1=iiwa;
		client=client_in;
		FingerTip=FingerTip_in;
		ToolGripper=ToolGripper_in;
	}
	
	public void init_startTime(long start_nanotime_input){
		start_nanotime=start_nanotime_input;
	}
	

	public String[] getString(){
		String[] jointStr;
		synchronized(this){
			jointStr = recvStr.split(" ");
			//System.out.printf("receive length data:%d\n" , jointStr.length);
			LastAccess = System.currentTimeMillis();
		}
		return jointStr;
	}
	

//	public YW_UDP_Server_MT_collector_datatorque() throws IOException{
	//	this("UDPTestThread");
	//}
	
	//public YW_UDP_Server_MT_collector_datatorque(String name) throws IOException{
	//	super(name);
	//}
	
	public void kill()
	{
		end = true;
	}

	public void close(){
		close=true;
	}
    boolean hold=false;
    void hold(){
    	hold=true;
    }
    void release_hold(){
    	hold=false;
    }
	public void run() {
		double ForceX;
		double ForceY;
		double ForceZ;
		double HapticForceX;
		double HapticForceY;
		double HapticForceZ;
		double TorqueX;
		double TorqueY; 
		double TorqueZ; 
		int gripper_value=255;
		double[] last_JointsDouble_sp={0.123,0.0,0.0,0.0,0.0,0.0,0.0};
		boolean stopj_done=false;
		double RB1_x = 0.0 ;
		double RB1_y = 0.0 ;
		double RB1_z = 0.0;
		double RB1_oa = 0.0 ;
		double RB1_ob = 0.0 ;
		double RB1_og = 0.0 ;
		double RB1_qx = 0.0 ;
		double RB1_qy = 0.0 ;
		double RB1_qz = 0.0 ;
		double RB1_qw = 0.0 ;
		double Old_RB1_x = 0.0 ;
		double Old_RB1_y = 0.0 ;
		double Old_RB1_z = 0.0 ;
		double Old_RB1_oa = 0.0 ;
		double Old_RB1_ob = 0.0 ;
		double Old_RB1_og = 0.0 ;
		double Old_RB1_qx = 0.0 ;
		double Old_RB1_qy = 0.0 ;
		double Old_RB1_qz = 0.0 ;
		double Old_RB1_qw = 0.0 ;
		double Old_Frame_x = 0.0 ;
		double Old_Frame_y = 0.0 ;
		double Old_Frame_z = 0.0 ;
		double Old_Frame_oa = 0.0 ;
		double Old_Frame_ob = 0.0 ;
		double Old_Frame_og = 0.0 ;
		double Org_X = 0.0; 
		double Org_Y = 0.0; 
		double Org_Z = 0.0; 
		double Org_a = 0.0; 
		double Org_b = 0.0; 
		double Org_g = 0.0; 
		int SetPointOutOfRange=0;
		Frame currentFrame = new Frame(); 
		int robotPosition=0; 
		int fingerTipForce=0; 
		int var_len;
		int var_start;
		while (!client.hasReceived)		// wait for data to be received before proceeding with the rest of the code
			ThreadUtil.milliSleep(500);
		   while(! end){
			   
			   while( hold){
				   ThreadUtil.milliSleep(1000);
				   System.out.println("DB Holding");
			   }
			   communicator_loop+=1;
			   //System.out.println("Communicator One loop+1"+communicator_loop);
			   ThreadUtil.milliSleep(10);
			   
			   
			   //get data
				long startTime = System.nanoTime();
				//theServoRuntime.updateWithRealtimeSystem();
				String[] jointStr = client.getString(); // get data from client 
				//double[] MsgDouble=new double[jointStr.length]; 
				

					//decode the msg
					//int ControlMode=Integer.valueOf(jointStr[0]);
					ControlMode=Integer.valueOf(jointStr[0]);
					//double[] xyzabgDouble_sp=new double[7]; 
					for(int j=1;j<8;j++)
					{
						xyzabgDouble_sp[j-1]=Double.valueOf(jointStr[j]);
						//System.out.print("receive float data: "+jointDouble[j] + ", ");
					}
					
					//double[] JointsDouble_sp=new double[7];
					for(int j=8;j<15;j++){
						int i=j-8;
						JointsDouble_sp[i]=Double.valueOf(jointStr[j]);
					}
					
					//double[] ForceDouble_sp=new double[6];
					for(int j=15;j<21;j++){
						int i=j-15;
						ForceDouble_sp[i]=Double.valueOf(jointStr[j]);
					}
					
					SpeedDouble_sp=Double.valueOf(jointStr[21]);
					//double SpeedDouble_sp=Double.valueOf(jointStr[21]);
					
					double[] stiff_sp=new double[6];
					for(int j=22;j<28;j++){
						int i=j-22;
						stiff_sp[i]=Double.valueOf(jointStr[j]);
					}
					
					var_len=7;
					var_start=28;
					//double[] JointsSpeed_sp=new double[var_len];
					for(int j=var_start;j<var_start+var_len;j++){
						int i=j-var_start;
						JointsSpeed_sp[i]=Double.valueOf(jointStr[j]);
					}
					
					//double blending_sp=Double.valueOf(jointStr[35]);
					blending_sp=Double.valueOf(jointStr[35]);
					
					var_len=6;
					var_start=36;
					//double[] MaxForce_sp=new double[var_len];
					for(int j=var_start;j<var_start+var_len;j++){
						int i=j-var_start;
						MaxForce_sp[i]=Double.valueOf(jointStr[j]);
					}
					
					var_len=6;
					var_start=42;
					//double[] MaxForce_sp=new double[var_len];
					for(int j=var_start;j<var_start+var_len;j++){
						int i=j-var_start;
						TCPxyzabgDouble_sp[i]=Double.valueOf(jointStr[j]);
					}
					
					
					
				//send data
					ForceSensorData torData = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getFrame("/BasePad/BasePadPinch_FF"));
					currentFrame = lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolGripper.getFrame("/BasePad"));
					
					fingerTipForce = FingerTip.getForce(); // force will be set force, which is 5 
					torData = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getFrame("/BasePad/BasePadPinch_FF"));
					// Original
					// torData = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getFrame("/BasePad"));
					
					ForceX = torData.getForce().getX();
					ForceY = torData.getForce().getY();
					ForceZ = torData.getForce().getZ();
					
					// My Original
					HapticForceX = -1.0*ForceZ;
					HapticForceY = -1.0*ForceY;
					HapticForceZ = 1.0*ForceX;
					currentFrame = lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolGripper.getFrame("/BasePad"));
					
					RB1_x = currentFrame.getX();
					RB1_y = currentFrame.getY();
					RB1_z = currentFrame.getZ();
					RB1_oa = currentFrame.getAlphaRad();
					RB1_ob = currentFrame.getBetaRad();
					RB1_og = currentFrame.getGammaRad();
					gripper_value=FingerTip.getFingerPos();
					
					TorqueX = torData.getTorque().getX();
					TorqueY = torData.getTorque().getY();
					TorqueZ = torData.getTorque().getZ();

					robotPosition=0; 
					jointAngle = lbr_iiwa_14_R820_1.getCurrentJointPosition().get();

					String sendStr = new String(); 
					sendStr=String.valueOf(RB1_x)+" "+String.valueOf(RB1_y)+" "+String.valueOf(RB1_z)+" "
							+String.valueOf(RB1_oa)+" "+String.valueOf(RB1_ob)+" "+String.valueOf(RB1_og) +" "
							+String.valueOf(gripper_value)+" "
							+String.valueOf(SetPointOutOfRange) +" " 
							+ String.valueOf(HapticForceX)+" "+String.valueOf(HapticForceY)+" "+String.valueOf(HapticForceZ)+" "
							+ String.valueOf(TorqueX)+" "+ String.valueOf(TorqueY)+" "+ String.valueOf(TorqueZ)+" "
							//+sendStr.valueOf(CurrentPosition.get(0))+" "+sendStr.valueOf(CurrentPosition.get(1))+" "
	                		//+sendStr.valueOf(CurrentPosition.get(2))+" "+sendStr.valueOf(CurrentPosition.get(3))+" "
	                		//+sendStr.valueOf(CurrentPosition.get(4))+" "+sendStr.valueOf(CurrentPosition.get(5))+" "
	                		//+sendStr.valueOf(CurrentPosition.get(6))+" "
							
							+ String.valueOf(fingerTipForce) + " " + String.valueOf(robotPosition) + " "
							+ String.valueOf(jointAngle[0]) + " " + String.valueOf(jointAngle[1]) + " "
							+ String.valueOf(jointAngle[2]) + " " + String.valueOf(jointAngle[3]) + " "
							+ String.valueOf(jointAngle[4]) + " " + String.valueOf(jointAngle[5]) + " "
							+ String.valueOf(jointAngle[6]) + '\0';
						if (client.sendMsg(sendStr))
						{
							//System.out.println("Message sending succeeded"); 
						}
						else
						{
							System.out.println("Message sending failed"); 
						}
			   
		   } // while

		   
		   System.out.println("Communicator One loop");
		   
		   
		   //end
		   while(!close){
			   //hoding and do nothing
			   ThreadUtil.milliSleep(3000);
		   }
		   
	} // run 
} // class 