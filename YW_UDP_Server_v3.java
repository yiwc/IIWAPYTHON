package application;

import java.io.IOException; // <--
import java.util.Arrays;    // <--
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.TimeUnit;


import application.YW_UDP_Server_v2.RobotMode;

import com.kuka.generated.ioAccess.RobotiqGrip3IOGroup; // <--

// Default
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication; // <--
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;       // <--

import com.kuka.roboticsAPI.controllerModel.Controller;              // <--
import com.kuka.roboticsAPI.deviceModel.LBR;                         // <--


import com.kuka.roboticsAPI.conditionModel.ForceComponentCondition;
import com.kuka.roboticsAPI.conditionModel.TorqueComponentCondition;

import com.kuka.common.ThreadUtil;                                   // <--

import com.kuka.roboticsAPI.geometricModel.CartDOF;                  // <--
import com.kuka.roboticsAPI.geometricModel.Frame;                    // <--

import com.kuka.roboticsAPI.geometricModel.Tool;                     // <--

import com.kuka.roboticsAPI.geometricModel.World;                    // <--
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;      
import com.kuka.roboticsAPI.motionModel.IDirectServoRuntime;       // <--

import com.kuka.roboticsAPI.motionModel.SmartServo; 
import com.kuka.roboticsAPI.motionModel.DirectServo;               // <--
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode; // <--
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;             // <--
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;   // <--

import com.kuka.roboticsAPI.deviceModel.JointPosition;

import com.kuka.roboticsAPI.motionModel.IMotionContainer;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */

public class YW_UDP_Server_v3 extends RoboticsAPIApplication
{
	public enum RobotMode {unknown, normal, impedance, smart, direct};
	private IMotionContainer motionContainer = null;
	RobotMode lastRobotMode = RobotMode.unknown;
    SmartServo smartServo = null;
    ISmartServoRuntime smartMotion = null;
    DirectServo directServo = null;
    IDirectServoRuntime directMotion = null;
    
	private Controller kuka_Sunrise_Cabinet_1; 
	private Tool ToolGripper;
	private RobotiqGrip3IOGroup FingerTip;
	private LBR lbr_iiwa_14_R820_1;
	private CartesianImpedanceControlMode mode = null ; 
	private CartesianImpedanceControlMode modeHandShake = null ;
	public YW_UDPThread client = null; // thread
	public YW_UDP_Server_v3_communicator client_communicator=null;
	public YW_UDP_Server_v3_communicator db=null;
	byte[] receive_data = new byte[1024];
	byte[] send_data = new byte[1024]; 

	boolean done=false;
	boolean _IsRunning = true;
	Frame initFrame = new Frame();
	int robotPosition=0; 
	int fingerTipForce=0; 
	double[] jointAngle = new double[8];

	enum p_mode {free, centering};
	p_mode previousMode = p_mode.centering; 

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
	
	// Pinching mode
	int gripper_mode = 11;
	// Original
	// int gripper_mode = 9;


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
	
	public void initialize()
	{
		lbr_iiwa_14_R820_1 = ServoMotionUtilities.locateLBR(getContext());

		kuka_Sunrise_Cabinet_1 = (Controller) getContext().getControllers().toArray()[0];

		FingerTip= new RobotiqGrip3IOGroup(kuka_Sunrise_Cabinet_1);

		ToolGripper=getApplicationData().createFromTemplate("ToolGripper");	

		mode = new CartesianImpedanceControlMode() ; //cyw original 20 20 20
		mode.parametrize(CartDOF.A).setStiffness(300);  	// rotation about Z
		mode.parametrize(CartDOF.B).setStiffness(300);		// rotation about Y
		mode.parametrize(CartDOF.C).setStiffness(300);		// rotation about X
		// Original
		// mode.parametrize(CartDOF.A).setStiffness(10) ;  	// rotation about Z
		// mode.parametrize(CartDOF.B).setStiffness(10);		// rotation about Y
		// mode.parametrize(CartDOF.C).setStiffness(10);		// rotation about X
		
		// Increase stiffness (reduce compliance)
		mode.parametrize(CartDOF.X).setStiffness(2000) ;//cyw original 300 300 300
		mode.parametrize(CartDOF.Y).setStiffness(2000) ;
		mode.parametrize(CartDOF.Z).setStiffness(2000) ;
		// Original
		// mode.parametrize(CartDOF.X).setStiffness(100) ;
		// mode.parametrize(CartDOF.Y).setStiffness(100) ;
		// mode.parametrize(CartDOF.Z).setStiffness(100) ;
		
		mode.parametrize(CartDOF.ALL).setDamping(0.7) ;

		modeHandShake = new CartesianImpedanceControlMode() ; //cyw original 50 50 10 1000 1000 1000
		modeHandShake.parametrize(CartDOF.A).setStiffness(300) ;  	// rotation about Z
		modeHandShake.parametrize(CartDOF.B).setStiffness(300);		// rotation about Y
		modeHandShake.parametrize(CartDOF.C).setStiffness(300);		// rotation about X

		modeHandShake.parametrize(CartDOF.X).setStiffness(2000) ;
		modeHandShake.parametrize(CartDOF.Y).setStiffness(2000) ;
		modeHandShake.parametrize(CartDOF.Z).setStiffness(2000) ;		
		modeHandShake.parametrize(CartDOF.ALL).setDamping(0.7) ;

		System.out.println("Initialized (Pick and Place - Amplified 1.5, translation only, with translational force feedback)" );
	} // initialise

	@Override 
	public void dispose()
	{
		_IsRunning = false ; 
		System.out.println(" Closing Sockets in Dispose Block"); 

		if(client != null)
			client.kill();
		if(client_communicator != null){
			client_communicator.kill();
			client_communicator.close();}
		if(db != null)	
			db.close();
			
		super.dispose();
	} // dispose

	private void moveToInitialPosition_mt()
	{	
	//lbr_iiwa_14_R820_1.move(ptp( Math.toRadians(70.13), Math.toRadians(20.34), Math.toRadians(0.06), Math.toRadians(-70.72) , Math.toRadians(-80.82), Math.toRadians(-20.79), Math.toRadians(-61.09)).setJointVelocityRel(0.3));
		//lbr_iiwa_14_R820_1.move(ptp( Math.toRadians(0.0), Math.toRadians(0), Math.toRadians(0), Math.toRadians(0) , Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)).setJointVelocityRel(0.3));
		//lbr_iiwa_14_R820_1.move(ptp( Math.toRadians(0.0), Math.toRadians(0), Math.toRadians(0), Math.toRadians(0) , Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)).setJointVelocityRel(0.3));
		lbr_iiwa_14_R820_1.move(ptpHome().setJointVelocityRel(0.25));
		if (FingerTip.getActReq() != 9)
		{
			FingerTip.setActReq(9);
		}
		FingerTip.setSpeed(255);
		//FingerTip.setForce(5);
		FingerTip.setPosReq(255); // close the gripper
		ThreadUtil.milliSleep(2000); 

	} // moveToInitialPosition_camera_catch_1
	// moveToInitialPosition_camera_catch_1
	private void moveToInitialPosition_camera_catch_1()
		{	
		//lbr_iiwa_14_R820_1.move(ptp( Math.toRadians(70.13), Math.toRadians(20.34), Math.toRadians(0.06), Math.toRadians(-70.72) , Math.toRadians(-80.82), Math.toRadians(-20.79), Math.toRadians(-61.09)).setJointVelocityRel(0.3));
		lbr_iiwa_14_R820_1.move(ptp( Math.toRadians(-20.13), Math.toRadians(31.34), Math.toRadians(0.06), Math.toRadians(-94.72) , Math.toRadians(-93.82), Math.toRadians(-57.79), Math.toRadians(-61.09)).setJointVelocityRel(0.3));
		
			if (FingerTip.getActReq() != gripper_mode)
			{
				FingerTip.setActReq(gripper_mode);
			}
			FingerTip.setSpeed(255);
			FingerTip.setForce(5);
			FingerTip.setPosReq(6); // close the gripper
			ThreadUtil.milliSleep(2000); 

		} // moveToInitialPosition_camera_catch_1
		
    void robotModeChange(RobotMode newMode){
    	//stop old motion servo
    	//ready for new motion servo
        if (motionContainer != null){
            if (lastRobotMode == RobotMode.smart){
                if (newMode != RobotMode.smart) smartMotion.stopMotion();
            }
            if (lastRobotMode == RobotMode.direct){
                if (newMode != RobotMode.direct) directMotion.stopMotion();
            }
        }
        lastRobotMode = newMode;
    }
	@SuppressWarnings("deprecation")
	@Override
	public void run() {

		try {

			ToolGripper.attachTo(lbr_iiwa_14_R820_1.getFlange());
			moveToInitialPosition_camera_catch_1(); 
			initFrame = lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolGripper.getFrame("/BasePad"));
			System.out.println("Starting RealtimeMotion in Position Mode");
			Frame goalFrame = new Frame(); 
			Frame currentFrame = new Frame(); 
//			ForceSensorData torData = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getRootFrame());
			ForceSensorData torData = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getFrame("/BasePad/BasePadPinch_FF"));
			// ForceSensorData torData = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getFrame("/BasePad/BasePadPinch_FF"));
			
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

			client = new YW_UDPThread(); 
			client.start(); // start the thread "UDPTestThread.java" 
			client_communicator = new YW_UDP_Server_v3_communicator();
			client_communicator.init(lbr_iiwa_14_R820_1, client,FingerTip,ToolGripper);
			client_communicator.start();
			db=client_communicator;
			
			System.out.println("Client has 15 seconds to start sending data" );
			while (!client.hasReceived){		// wait for data to be received before proceeding with the rest of the code
				System.out.println("Wait for receive..." );
				ThreadUtil.milliSleep(1000);
			}
			System.out.println("Go into the while" );

			double[] last_JointsDouble_sp={0.123,0.0,0.0,0.0,0.0,0.0,0.0};
			double[] last_JointsDouble_sp_init={0.123,0.0,0.0,0.0,0.0,0.0,0.0};
			boolean stopj_done=false;

			int var_len;
			int var_start;

			int lastControlMode=0;
			while(!done)
			{	
				
				//System.out.println("Main Loop +1"+" son get communicator:"+client_communicator.communicator_loop);
				ThreadUtil.milliSleep(10);
				
				//get data
				long startTime = System.nanoTime();
				ControlMode=db.ControlMode;
				xyzabgDouble_sp=db.xyzabgDouble_sp;
				JointsDouble_sp=db.JointsDouble_sp;
				ForceDouble_sp=db.ForceDouble_sp;
				SpeedDouble_sp=db.SpeedDouble_sp;
				stiff_sp=db.stiff_sp;
				JointsSpeed_sp=db.JointsSpeed_sp;
				blending_sp=db.blending_sp;
				MaxForce_sp=db.MaxForce_sp;
				TCPxyzabgDouble_sp=db.TCPxyzabgDouble_sp;
			 
				//do command
				ThreadUtil.milliSleep(1000);
				System.out.println("Main Loop Get test: ComtrolMode="+ControlMode);
				System.out.println("xyzabgDouble_sp"+xyzabgDouble_sp[0]+","+xyzabgDouble_sp[1]+","+xyzabgDouble_sp[2]+","+xyzabgDouble_sp[3]+","+xyzabgDouble_sp[4]+","+xyzabgDouble_sp[5]+","+xyzabgDouble_sp[6]+",");
				System.out.println("TCPxyzabgDouble_sp"+TCPxyzabgDouble_sp[0]+","+TCPxyzabgDouble_sp[1]+","+TCPxyzabgDouble_sp[2]+","+TCPxyzabgDouble_sp[3]+","+TCPxyzabgDouble_sp[4]+","+TCPxyzabgDouble_sp[5]+",");
				System.out.println("JointsDouble_sp"+JointsDouble_sp[0]+","+JointsDouble_sp[1]+","+JointsDouble_sp[2]+","+JointsDouble_sp[3]+","+JointsDouble_sp[4]+","+JointsDouble_sp[5]+","+JointsDouble_sp[6]+",");
				//System.out.println("ForceDouble_sp"+ForceDouble_sp[0]+","+ForceDouble_sp[1]+","+ForceDouble_sp[2]+","+ForceDouble_sp[3]+","+ForceDouble_sp[4]+","+ForceDouble_sp[5]+",");
				//System.out.println("SpeedDouble_sp"+SpeedDouble_sp);
				//System.out.println("JointsSpeed_sp"+JointsSpeed_sp[0]+","+JointsSpeed_sp[1]+","+JointsSpeed_sp[2]+","+JointsSpeed_sp[3]+","+JointsSpeed_sp[4]+","+JointsSpeed_sp[5]+","+JointsSpeed_sp[6]+",");
				
				FingerTip.setPosReq((int)(xyzabgDouble_sp[6])); 	// max value of jointDouble[12]*496 is 255
				
				if(ControlMode!=9){
					stopj_done=false;
				}
				
				if(ControlMode==0){
					//Smart Servo Controller
					if (smartServo == null) {
						smartServo = new SmartServo(lbr_iiwa_14_R820_1.getCurrentJointPosition());
						smartServo.setTimeoutAfterGoalReach(100);
						
					}

		            if (lastRobotMode != RobotMode.smart){
		                if (motionContainer != null) motionContainer.cancel();
		                motionContainer = ToolGripper.getFrame("/BasePad").moveAsync(smartServo);
		                smartMotion = smartServo.getRuntime();
		            }
		            //ToolGripper.getFrame("/BasePad").moveAsync(aSmartServoMotion.setMode(mode).setJointVelocityRel(1.0));
					
					
					goalFrame = lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolGripper.getFrame("/BasePad"));
					//aSmartServoMotion.setJointVelocityRel(0.1);
						
					//Org_X = goalFrame.getX(); 
					//Org_Y = goalFrame.getY(); 
					//Org_Z = goalFrame.getZ(); 
					//Org_a = goalFrame.getAlphaRad(); 
					//Org_b = goalFrame.getBetaRad(); 
					//Org_g = goalFrame.getGammaRad();  
					goalFrame.setX(xyzabgDouble_sp[0]); // scaling by factor 1 
					goalFrame.setY(xyzabgDouble_sp[1]); // scaling by factor 1
					goalFrame.setZ(xyzabgDouble_sp[2]); // scaling by factor 1
					goalFrame.setAlphaRad(xyzabgDouble_sp[3]);
					goalFrame.setBetaRad(xyzabgDouble_sp[4]);
					goalFrame.setGammaRad(xyzabgDouble_sp[5]);

	                
					try{
						smartMotion.setDestination(goalFrame, World.Current.getRootFrame());
						SetPointOutOfRange=0;
					}
					catch (Exception err) {
						//err.printStackTrace();
						SetPointOutOfRange=1;
						System.out.println("setpoint out of range or"+err);
						ThreadUtil.milliSleep(1000);
						
					}
					lastRobotMode = RobotMode.smart;
					
				}
				
				else if(ControlMode==1){
					// Smart Servo Joints Control
					
		            if (smartServo == null) {
		            	smartServo = new SmartServo(lbr_iiwa_14_R820_1.getCurrentJointPosition());
						smartServo.setTimeoutAfterGoalReach(100);
		            }

		            if (lastRobotMode != RobotMode.smart){
		                if (motionContainer != null) motionContainer.cancel();
		                motionContainer = ToolGripper.getFrame("/BasePad").moveAsync(smartServo);
		                smartMotion = smartServo.getRuntime();
		            }

		                JointPosition jointPosition = new JointPosition(
		                		JointsDouble_sp[0],
		                		JointsDouble_sp[1],
		                		JointsDouble_sp[2],
		                		JointsDouble_sp[3],
		                		JointsDouble_sp[4],
		                		JointsDouble_sp[5],
		                		JointsDouble_sp[6]);

		                JointPosition jointSpeed = new JointPosition(
		                		JointsSpeed_sp[0],
		                		JointsSpeed_sp[1],
		                		JointsSpeed_sp[2],
		                		JointsSpeed_sp[3],
		                		JointsSpeed_sp[4],
		                		JointsSpeed_sp[5],
		                		JointsSpeed_sp[6]
		                		);

		               // smartMotion.setDestination(jointPosition, jointSpeed);
		                if(Arrays.equals(last_JointsDouble_sp,JointsDouble_sp)&& lastControlMode==1 ){
		                	//System.out.println("joints equal");
		                }
		                else{
		               	try{
		                	//System.out.println("Joint Controlling..");
		                	smartMotion.setDestination(jointPosition, jointSpeed);
		                	}
		                	catch (Exception err){
								//err.printStackTrace();
								ThreadUtil.milliSleep(1000);
								System.out.println("joint control error"+err);
		                	}
		                	last_JointsDouble_sp=JointsDouble_sp.clone();
		                }
		                
		                lastRobotMode = RobotMode.smart;


					//Smart Servo For joints
					
				}
				
				else if(ControlMode==2){
					//Force Empedence Controller
					
		            robotModeChange(RobotMode.impedance);
		            	goalFrame = lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolGripper.getFrame("/BasePad"));
		                Frame frame = goalFrame;
		                
		                frame.setX(xyzabgDouble_sp[0]);
		                frame.setY(xyzabgDouble_sp[1]);
		                frame.setZ(xyzabgDouble_sp[2]);
		                frame.setAlphaRad(xyzabgDouble_sp[3]);
		                frame.setBetaRad(xyzabgDouble_sp[4]);
		                frame.setGammaRad(xyzabgDouble_sp[5]);
		                
		                
		                CartesianImpedanceControlMode impedance = new CartesianImpedanceControlMode();
		                impedance.parametrize(CartDOF.X).setStiffness(stiff_sp[0]);
		                impedance.parametrize(CartDOF.Y).setStiffness(stiff_sp[1]);
		                impedance.parametrize(CartDOF.Z).setStiffness(stiff_sp[2]);
		                impedance.parametrize(CartDOF.A).setStiffness(stiff_sp[3]);
		                impedance.parametrize(CartDOF.B).setStiffness(stiff_sp[4]);
		                impedance.parametrize(CartDOF.C).setStiffness(stiff_sp[5]);
		                impedance.parametrize(CartDOF.X).setAdditionalControlForce(ForceDouble_sp[0]);
		                impedance.parametrize(CartDOF.Y).setAdditionalControlForce(ForceDouble_sp[1]);
		                impedance.parametrize(CartDOF.Z).setAdditionalControlForce(ForceDouble_sp[2]);
		                impedance.parametrize(CartDOF.A).setAdditionalControlForce(ForceDouble_sp[3]);
		                impedance.parametrize(CartDOF.B).setAdditionalControlForce(ForceDouble_sp[4]);
		                impedance.parametrize(CartDOF.C).setAdditionalControlForce(ForceDouble_sp[5]);
		                //impedance.setAdditionalControlForce(-forceX, -forceY, -forceZ, -torqueX, -torqueY, -torqueZ);
		                
		                
		                if (motionContainer != null) motionContainer.cancel();
		                motionContainer = ToolGripper.moveAsync(ptp(frame).setJointVelocityRel(SpeedDouble_sp).setMode(impedance));
		                
		                //motionContainer = tool.moveAsync(lin(frame).setJointVelocityRel(speed).setMode(impedance));
		                
				}
				else if(ControlMode==3){

		            if (lastRobotMode != RobotMode.normal){
		                if (motionContainer != null) motionContainer.cancel();
		            }
		            robotModeChange(RobotMode.normal);
		            Frame frame = lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolGripper.getFrame("/BasePad"));
	                frame.setX(xyzabgDouble_sp[0]);
	                frame.setY(xyzabgDouble_sp[1]);
	                frame.setZ(xyzabgDouble_sp[2]);
	                frame.setAlphaRad(xyzabgDouble_sp[3]);
	                frame.setBetaRad(xyzabgDouble_sp[4]);
	                frame.setGammaRad(xyzabgDouble_sp[5]);
	                //Frame frame = lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolGripper.getFrame("/BasePad"));
	                //frame.setX(TCPxyzabgDouble_sp[0]);
	                //frame.setY(TCPxyzabgDouble_sp[1]);
	                //frame.setZ(TCPxyzabgDouble_sp[2]);
	                //frame.setAlphaRad(TCPxyzabgDouble_sp[3]);
	                //frame.setBetaRad(TCPxyzabgDouble_sp[4]);
	                //frame.setGammaRad(TCPxyzabgDouble_sp[5]);
	                
	                //System.out.println("frame:"+frame.getX()+",  "+frame.getY()+",  "+frame.getZ()+",  "+frame.getAlphaRad()+",  "+frame.getBetaRad()+",  "+	frame.getGammaRad());
	                
	                
	                double speed = SpeedDouble_sp;
	                double blending = blending_sp;
	                double thresholdX = MaxForce_sp[0]; // Force threshold X
	                double thresholdY = MaxForce_sp[1]; // Force threshold Y
	                double thresholdZ = MaxForce_sp[2]; // Force threshold Z
	                double thresholdA = MaxForce_sp[3]; // Force threshold X
	                double thresholdB = MaxForce_sp[4]; // Force threshold Y
	                double thresholdC = MaxForce_sp[5]; // Force threshold Z
	                double offsetX = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getDefaultMotionFrame()).getForce().getX(); // Force offset X
	                double offsetY = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getDefaultMotionFrame()).getForce().getY(); // Force offset Y
	                double offsetZ = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getDefaultMotionFrame()).getForce().getZ(); // Force offset Z
	                double offsetA = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getDefaultMotionFrame()).getTorque().getZ(); // Force offset Z
	                double offsetB = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getDefaultMotionFrame()).getTorque().getY(); // Force offset Z
	                double offsetC = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getDefaultMotionFrame()).getTorque().getX(); // Force offset Z
	                //ForceComponentCondition conditionX = new ForceComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.X, offsetX - thresholdX, offsetX + thresholdX);
	                //ForceComponentCondition conditionY = new ForceComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.Y, offsetY - thresholdY, offsetY + thresholdY);
	               // ForceComponentCondition conditionZ = new ForceComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.Z, offsetZ - thresholdZ, offsetZ + thresholdZ);
	               // TorqueComponentCondition conditionC = new TorqueComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.X, offsetC - thresholdC, offsetC + thresholdC);
	               // TorqueComponentCondition conditionB = new TorqueComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.Z, offsetB - thresholdB, offsetB + thresholdB);
	               // TorqueComponentCondition conditionA = new TorqueComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.Z, offsetA - thresholdA, offsetA + thresholdA);
	                ForceComponentCondition conditionX = new ForceComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.X, -thresholdX, thresholdX);
	                ForceComponentCondition conditionY = new ForceComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.Y, -thresholdY, thresholdY);
	                ForceComponentCondition conditionZ = new ForceComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.Z, -thresholdZ, thresholdZ);
	                TorqueComponentCondition conditionC = new TorqueComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.X, -thresholdC, thresholdC);
	                TorqueComponentCondition conditionB = new TorqueComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.Z, -thresholdB, thresholdB);
	                TorqueComponentCondition conditionA = new TorqueComponentCondition(ToolGripper.getDefaultMotionFrame(), CoordinateAxis.Z, -thresholdA, thresholdA);
	                
	                System.out.println("threshold XYZABC,"+MaxForce_sp[0]+" "+MaxForce_sp[1]+" "+MaxForce_sp[2]+" "+MaxForce_sp[3]+" "+MaxForce_sp[4]+" "+MaxForce_sp[5]);
	                System.out.println("conditon XYZABC,"+(offsetX - thresholdX)+" "+(offsetY - thresholdY)+" "+(offsetZ - thresholdZ)+" "+(offsetA - thresholdA)+" "+(offsetB - thresholdB)+" "+(offsetC - thresholdC));
	                
	                
	                //motionContainer = ToolGripper.moveAsync(ptp(frame).setJointVelocityRel(speed).setBlendingCart(blending).breakWhen(conditionX).breakWhen(conditionY).breakWhen(conditionZ).breakWhen(conditionA).breakWhen(conditionB).breakWhen(conditionC));
	                //motionContainer = ToolGripper.moveAsync(ptp(frame).setJointVelocityRel(speed).breakWhen(conditionX).breakWhen(conditionY).breakWhen(conditionZ).breakWhen(conditionA).breakWhen(conditionB).breakWhen(conditionC));
	                //for(int i=0;i<15;i++){
	                //	System.out.println("ptp k="+i);
	                //motionContainer = ToolGripper.moveAsync(ptp(frame).setJointVelocityRel(speed).breakWhen(conditionA).breakWhen(conditionB).breakWhen(conditionC));

		              // motionContainer = ToolGripper.getFrame("/BasePad").moveAsync(ptp(frame).setJointVelocityRel(speed).breakWhen(conditionX).breakWhen(conditionY).breakWhen(conditionZ));

		               motionContainer = ToolGripper.getFrame("/BasePad").moveAsync(ptp(frame).setJointVelocityRel(speed).breakWhen(conditionX).breakWhen(conditionY).breakWhen(conditionZ).breakWhen(conditionA).breakWhen(conditionB).breakWhen(conditionC));
	                
		                //motionContainer = ToolGripper.moveAsync(ptp(frame).setJointVelocityRel(0.25).setBlendingCart(0.5));
		                
	                //}
	                
				}

				else if(ControlMode==9){
					// Stop Joints movement
		            if (smartServo == null) smartServo = new SmartServo(lbr_iiwa_14_R820_1.getCurrentJointPosition());

		            if (lastRobotMode != RobotMode.smart){
		                if (motionContainer != null) motionContainer.cancel();
		                motionContainer = ToolGripper.getFrame("/BasePad").moveAsync(smartServo);
		                smartMotion = smartServo.getRuntime();
		               
		            }

		                JointPosition jointPosition = new JointPosition(lbr_iiwa_14_R820_1.getCurrentJointPosition().get());

		                JointPosition jointSpeed = new JointPosition(
		                		JointsSpeed_sp[0],
		                		JointsSpeed_sp[1],
		                		JointsSpeed_sp[2],
		                		JointsSpeed_sp[3],
		                		JointsSpeed_sp[4],
		                		JointsSpeed_sp[5],
		                		JointsSpeed_sp[6]
		                		);
		                //smartMotion.stopMotion();
		                robotModeChange(RobotMode.unknown);
		                if(stopj_done){
		                	//smartMotion.stopMotion();
		                }
		                else{
		                	//smartMotion.stopMotion();
		                	System.out.println("Stop Joint Controlling..");
		                	//smartMotion.setDestination(jointPosition, jointSpeed);
		                	stopj_done=true;
		                }
		                //lastRobotMode = RobotMode.smart;
					//Smart Servo For joints
				}

				lastControlMode=ControlMode;
				
				
				
				continue;
				
				
				
			} // while
			System.out.println("main loop finished");
		}// try 

		catch (IOException e1) {
			e1.printStackTrace();
		}
	} // run 

	public static void main(String[] args) {
		YW_UDP_Server_v3 app = new YW_UDP_Server_v3();
		app.runApplication();
	}

} // class
