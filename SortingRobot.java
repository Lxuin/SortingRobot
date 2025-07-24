package MZR_25;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.InetSocketAddress;
import java.net.Socket;
import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import java.io.PrintWriter;
import utils.Gripper;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.geometricModel.Frame;
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
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */



public class SortingRobot extends RoboticsAPIApplication {

    //Instanzieren des Objektes Iiwa
    @Inject
    public LBR iiwa;

    //Instanzieren des Objektes Greifer
    @Inject
    private Gripper grip;

    public static Socket socket;

    public static PrintWriter writer;

    public static BufferedReader reader;

    private volatile boolean running = false;

    //Definition der anzufahrenden Punkte
    Frame start;
    Frame grabPre;
    Frame grab;
    Frame overCollision;
    Frame collisionPre;
    Frame collision;
    Frame boxOne;
    Frame boxOneDrop;
    Frame boxTwo;
    Frame boxTwoDrop;
    Frame boxThree;
    Frame boxThreeDrop;

    @Override
    public void initialize() {
        //Ausgabe eines Startsignals aus dem Handbdienger�t
        getLogger().info("Die Applikation startet!");
        //Greifer initialisieren
        iiwa = getContext().getDeviceFromType(LBR.class);
        openGripper();

        start = getApplicationData().getFrame("/Start").copyWithRedundancy();
        iiwa.move(ptp(start).setJointVelocityRel(0.5));
    }

    @Override
    public void run() throws Exception {
        //Definition der anzufahrenden Punkte
        grabPre = getApplicationData().getFrame("/GrabPre").copyWithRedundancy();
        grab = getApplicationData().getFrame("/Grab").copyWithRedundancy();
        overCollision = getApplicationData().getFrame("/OverCollision").copyWithRedundancy();
        collisionPre = getApplicationData().getFrame("/CollisionPre").copyWithRedundancy();
        collision = getApplicationData().getFrame("/Collision").copyWithRedundancy();
        boxOne = getApplicationData().getFrame("/BoxOne").copyWithRedundancy();
        boxOneDrop = getApplicationData().getFrame("/BoxOneDrop").copyWithRedundancy();
        boxTwo = getApplicationData().getFrame("/BoxTwo").copyWithRedundancy();
        boxTwoDrop = getApplicationData().getFrame("/BoxTwoDrop").copyWithRedundancy();
        boxThree = getApplicationData().getFrame("/BoxThree").copyWithRedundancy();
        boxThreeDrop = getApplicationData().getFrame("/BoxThreeDrop").copyWithRedundancy();

        StaticSocketClient.connect("172.31.1.74", 30001);

        new Thread(new SocketListenerThread()).start();

        JointTorqueCondition cond = new JointTorqueCondition(JointEnum.J2, 18, 21);

        boolean screwsLeft = true;

        while(screwsLeft) {
            while(!running) {
                ThreadUtil.milliSleep(100);
            }
            StaticSocketClient.rechts1();

            StaticSocketClient.sayGreifen();

            iiwa.move(ptp(grabPre).setJointVelocityRel(0.5));
            iiwa.move(lin(grab).setJointVelocityRel(0.3));
            closeGripper();

            iiwa.move(lin(grabPre).setJointVelocityRel(0.3));

            StaticSocketClient.rechts2();

            iiwa.move(lin(overCollision).setJointVelocityRel(0.5));

            iiwa.move(ptp(collisionPre).setJointVelocityRel(0.3));

            iiwa.move(lin(collision).setJointVelocityRel(0.02).breakWhen(cond));

            double zCord = iiwa.getFlange().getZ();

            int box;

            if(zCord > 287 && zCord < 289) {
                getLogger().info("kleine Schraube");
                StaticSocketClient.saySchraubeKlein();
                box = 1;
            }
            else if(zCord > 301.0 && zCord < 303.0) {
                getLogger().info("mittlere Schraube");
                StaticSocketClient.saySchraubeRund();
                box = 2;
            }
            else if(zCord > 322.0 && zCord < 324.5) {
                getLogger().info("große Schraube");
                StaticSocketClient.saySchraubeGross();
                box = 3;
            }
            else {
                getLogger().info("keine Schraube");
                box = 0;
                screwsLeft = false;
            }

            StaticSocketClient.rechts3();
            iiwa.move(lin(overCollision).setJointVelocityRel(0.5));

            if(box == 1) {
                iiwa.move(ptp(boxOne).setJointVelocityRel(0.5));
                iiwa.move(lin(boxOneDrop).setJointVelocityRel(0.3));
                openGripper();
                StaticSocketClient.sayBoxKlein();
                iiwa.move(ptp(boxOne).setJointVelocityRel(0.5));
            }
            else if (box == 2) {
                iiwa.move(ptp(boxTwo).setJointVelocityRel(0.5));
                iiwa.move(lin(boxTwoDrop).setJointVelocityRel(0.3));
                openGripper();
                StaticSocketClient.sayBoxRund();
                iiwa.move(ptp(boxTwo).setJointVelocityRel(0.5));
            }
            else if (box == 3) { //box == 3
                iiwa.move(ptp(boxThree).setJointVelocityRel(0.5));
                iiwa.move(lin(boxThreeDrop).setJointVelocityRel(0.3));
                openGripper();
                StaticSocketClient.sayBoxGross();
                iiwa.move(ptp(boxThree).setJointVelocityRel(0.5));            }
        }

        StaticSocketClient.gerade();

        iiwa.move(ptp(start).setJointVelocityRel(0.5));

        StaticSocketClient.close();

        getLogger().info("Die Applikation ist beendet!");
    }

    public void openGripper(){
        if(!grip.isGripperOpen()) grip.openGripper();
        while(!grip.isGripperOpen()){
            ThreadUtil.milliSleep(100);
        }
    }

    public void closeGripper(){
        if(!grip.isGripperClosed()) grip.closeGripper();
        while(!grip.isGripperClosed()){
            ThreadUtil.milliSleep(100);
        }
    }

    private class SocketListenerThread implements Runnable {
        @Override
        public void run() {
            try {
                reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                String msg;
                while ((msg = reader.readLine()) != null) {
                    msg = msg.trim().toLowerCase();
                    if (msg.equals("start")) {
                        running = true;
                        getLogger().info("startet...");
                    }
                    else if (msg.equals("pause")) {
                        running = false;
                        getLogger().info("pausiert...");
                    }
                    else {
                        getLogger().info("unbekannte Nachricht: " + msg);
                    }
                }
            } catch (IOException e) {
                getLogger().warn("SocketListener Fehler: " + e.getMessage());
            }
        }
    }

     public static class StaticSocketClient {
            public static void connect(String ip, int port) {
                try {
                    socket = new Socket();
                    socket.connect(new InetSocketAddress(ip, port), 5000);
                    writer = new PrintWriter(socket.getOutputStream(), true);
                    System.out.println("Connected");
                } catch (IOException e) {
                    System.out.println("Connection failed: " + e.getMessage());
                }
            }

            private static void send(String msg) {
                if (writer != null) {
                    writer.println(msg);
                } else {
                    System.out.println("Writer not ready.");
                }
            }

            public static void saySchraubeGross()  { send("gs"); }
            public static void saySchraubeKlein()  { send("ks"); }
            public static void saySchraubeRund()   { send("rs"); }
            public static void sayKeineSchraube()  { send("ns");   }
            public static void sayBoxGross()       { send("gb"); }
            public static void sayBoxKlein()       { send("kb"); }
            public static void sayBoxRund()        { send("rb"); }
            public static void sayGreifen()        { send("g");  }
            public static void powerOn()           { send("on"); }
            public static void powerOff()          { send("off"); }
            public static void gerade()             { send("45"); }
            public static void rechts1()             { send("60"); }
            public static void rechts2()             { send("75"); }
            public static void rechts3()             { send("90"); }


            public static void close() {
                try {
                    if (writer != null) writer.close();
                    if (socket != null) socket.close();
                    System.out.println("Connection closed");
                } catch (IOException e) {
                    System.out.println("Close error: " + e.getMessage());
                }
            }
        }
    }