package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Scan;

public class Lidar extends SubsystemBase {

    /// Internal class that is used to run LidarIO processing on a separate thread:
    class LidarIO implements Runnable {
        private SerialPort serialPort;

        // Data packet parsing values:
        private final byte getInfo[] = {(byte) 0xa5, (byte) 0x52};
        private final byte scanDescriptor[] = {(byte) 0xa5, (byte) 0x5a, (byte) 0x05, (byte) 0x00, (byte) 0x00, (byte) 0x40, (byte) 0x81};
        private final byte stopCommand[] = {(byte) 0xa5, (byte) 0x25};
        private final byte startCommand[] = {(byte) 0xa5, (byte) 0x20};
        private final int bytesPerScan = 5;

        // Parsing state variable set after we get back descriptor.
        boolean measureMode = false;
        // Counter of received packets to be sure we have live data.
        int lidarUpdatesReceived = 0;

        // Filtering values:
        private final double minAcceptedRange = 0.09; // In meters
        private final double maxAcceptedRange = 2.2; // In meters
        private final double maxAcceptedX = 1.0;
        private final double minAcceptedY = -0.25;
        private final double maxAcceptedY = 0.39;
        private final int minAcceptedAngle1 = 0;   // In degrees, for the first range of accepted angles
        private final int maxAcceptedAngle1 = 45;  // In degrees, for the first range of accepted angles
        private final int minAcceptedAngle2 = 325; // In degrees, for the second range of accepted angles
        private final int maxAcceptedAngle2 = 360; // In degrees, for the second range of accepted angles
        private final int minAcceptedQuality = 5;
        private final double mountingOffset = 0.0; //in degrees


        // Transformation of points into robot coordinates:
        private final double x_offset = 0.289;
        private final double y_offset = 0.22;

        // Data storage for the data we've read:
        private ArrayList <Scan> scans = new ArrayList<Scan>();
        // Separate count of scans so we don't reallocate/resize the arraylist constantly:
        private int readScanCount = 0;

        // Output Variables protected by synchronize:
        private double xAvg = 0.0;
        private double slope = 0.0;
        private int scanCount = 0;
        private double scanTimestamp = 0.0; // Last completed scan timestamp.

        // Timing measurement variables:
        private double lastIterationTime = 0.0;


        LidarIO() {
            // Pre-allocate space for all scans, up to 250.
            scans.ensureCapacity(250);
            for (int ii = 0; ii < 250; ++ii) {
                scans.add(new Scan(0,0,0,0,0));
            }

        }

        /// Synchronized access to number of packets received.
        public synchronized int  getUpdatesReceived() {
            return lidarUpdatesReceived;
        }

        /// Synchronized access to latest timestamp of data.
        public synchronized double getScanTimestamp() {
            return scanTimestamp;
        }

        public synchronized int getScanCount() {
            return scanCount;
        }

        public synchronized double getAvgX() {
            return xAvg;
        }

        public synchronized double getSlope() {
            return slope;
        }

        public synchronized boolean isOK() {
            return serialPort != null;
        }

        public synchronized double getLastIterationTime() {
            return lastIterationTime;
        }

    
        @Override
        public void run() {
            try{
                serialPort = new SerialPort(460800, SerialPort.Port.kUSB1, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
                serialPort.setReadBufferSize(1024);
            }
            catch(Exception e){
                System.out.println("LidarIO: No LiDAR found");
                serialPort = null;
            }
            if (serialPort != null){
                serialPort.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
                serialPort.setFlowControl(SerialPort.FlowControl.kNone);
                System.out.println("LidarIO: handshaking...");
                Handshake();
                System.out.println("LidarIO: handshake done.");
            }

            // Main processing loop for Lidar:
            while (true) {
                double startTime =  Timer.getFPGATimestamp();
                // Check bytes available and run forever in loop....
                int numBytesAvail = serialPort.getBytesReceived();
                
                if (measureMode){
                    // read and parse all available scan data to read - fill array
                    readAndParseMeasurements(numBytesAvail);
                } else if (numBytesAvail >= 7) {
                    // read and check the first 7 bytes of response
                    if(parseDescriptor()){
                        // expected descriptor received, switch to read data
                        measureMode = true;
                    }
                    else{
                        System.out.println("LidarIO: Lidar handshake error");
                    }
                }
                double endTime = Timer.getFPGATimestamp();
                // Measure how we are doing on each iteration:
                setLastIterationTime(endTime - startTime);

                // Brief sleep:
                try {
                    Thread.sleep(10); // 100Hz max.
                }
                catch (Exception e) {
                    System.out.println(e);
                }
            }
        }

        private void Handshake () {
            System.out.println("LidarIO: Beginning of handshake method for Lidar sensor");
    
            // send stop
            serialPort.write(stopCommand, stopCommand.length);
            try {
                Thread.sleep(50);
            }
            catch (Exception e) {
                System.out.println(e);
            }
            serialPort.flush();
            serialPort.reset();
            // ?? add GET_HEALTH Request?
            // startCommand
            serialPort.write(startCommand, startCommand.length);
            System.out.println("LidarIO: Sent start command to Lidar sensor");
            // wait loop, while getBytesReceived is less, sleep
            int counter  = 0;
            while(serialPort.getBytesReceived() < 7 && counter < 15){
                try {
                    Thread.sleep(50);
                }
                catch (Exception e) {
                    System.out.println(e);
                }
                counter ++;
            }
    
            if (counter >= 15){
                serialPort.close();
                serialPort = null;
                System.out.println("LidarIO: Failed to handshake with lidar... disable...");
            }
            
            // How to print out hex in java - String.format("0x%02x", what you're printing out)
        }

        private boolean parseDescriptor(){
            byte[] received = serialPort.read(7);
            return Arrays.equals(received, scanDescriptor);
        }

        public void readAndParseMeasurements(int numBytesAvail){
            // round down to determine number of full scans available
            int numScansToRead = numBytesAvail/bytesPerScan;
            byte[] rawData = serialPort.read(numScansToRead * bytesPerScan);
            for(int i = 0; i < numScansToRead; i ++) {
                int offset = i * bytesPerScan;
                boolean recordScan = true; // Starts as true.
                if ((rawData[offset] & 0x003) == 1) {

                    // If we have enough, then process scan and update output.
                    if (readScanCount > 20) {
                        // This method has a synchronized block to store data for external threads to access.
                        calculateOutput(readScanCount, Timer.getFPGATimestamp()); // Updates outputs based on scan we have right now.
                    }
                    // Always start over accumulating new data for next time:
                    readScanCount = 0;
                } 
                
                // divide by 4 to drop the lower two bits
                int quality = ((rawData[offset + 0] & 0x0FC) >> 2);
                if (quality < minAcceptedQuality) recordScan = false;
                // angle = Math.pow(2, 7) * angle[14:7] + angle[6:0]
                float angle_deg = ((Byte.toUnsignedInt(rawData[offset + 2]) & 0x0FF) << 7) | ((Byte.toUnsignedInt(rawData[offset + 1]) & 0x0FE) >> 1);
                angle_deg /= 64.0f + mountingOffset;
                // range = Math.pow(2, 8) * distance[15:8] + distance[7:0]
                float range_mm = ((rawData[offset + 4] & 0x0FF) << 8) | (rawData[offset + 3] & 0x0FF);
                float angle_rad = 3.141592f * angle_deg / 180.0f;
                float range_m = (range_mm / 4.0f) / 1000;

                // Cartesian point in lidar coordinates:
                double x_l = Math.cos(-angle_rad) * range_m;
                double y_l = Math.sin(-angle_rad) * range_m;
               
                // Cartesian endpoint in robot coordinates:
                double x_robot = x_l + x_offset;
                double y_robot = y_l + y_offset;

                // Quality, range, and angle filter
                if (isAngleGood(angle_deg) == false) recordScan = false;
                if (isRangeGood(range_m) == false) recordScan = false;
                if (!isXInRange(x_robot)) recordScan = false;
                if (!isYInRange(y_robot)) recordScan = false;

                // If the scan is good, then record it:
                if(recordScan) {
                    // If we have space to record it:
                    if (readScanCount < 250) {
                        var scan = scans.get(readScanCount);
                        // Fill out existing scan:
                        scan.range = range_m;
                        scan.angle = angle_rad;
                        scan.quality = quality;
                        scan.x_robot = x_robot;
                        scan.y_robot = y_robot;
                        readScanCount++; // Count this scan for array one.
                    }
                } // Else we don't record this one... it gets filtered out.
            }
        }

        private void calculateOutput(int newScanCount, double newTimestamp){
            int count = 0;
            double s = 0;
            double ssum = 0;
            double localXAvg = 0.0;
            double localSlope = 0.0;

            for(int i = 0; i < scanCount; i++){
                localXAvg += scans.get(i).getX();
            }
            localXAvg /= scanCount;  // Externally visible synchronized variable.

            for(int i = 0; i < scanCount - 3; i++){
                var scan = scans.get(i);
                var scan2 = scans.get(i+2);

                if(Math.abs(scan2.getY() - scan.getY()) > 0.0001){
                    s = (scan2.getX() - scan.getX()) / (scan2.getY() - scan.getY());
                    if (s < 5.0 && s > -5.0){
                        ssum += s;
                        count++;
                    }
                }
            }
            localSlope = ssum / count; // Externally visible synchronized variable.

            // SET our other externally visible output variables inside this synchronized block.
            // Do as little work in a synchronized block as *possible*
            synchronized (this) {
                xAvg = localXAvg;
                slope = localSlope;
                scanCount = newScanCount;
                scanTimestamp = newTimestamp;
                lidarUpdatesReceived++; // Increment number of updates we've done
            }
        }

        private synchronized void setLastIterationTime(double t) {
            lastIterationTime = t;
        }
    
        private boolean isXInRange(double x){
            return x <= maxAcceptedX && x > 0;
        }
    
        private boolean isYInRange(double y){
            return y > minAcceptedY && y < maxAcceptedY;
        }
    
        private boolean isAngleGood(float angle){
            if((angle > minAcceptedAngle1 && angle < maxAcceptedAngle1) || (angle > minAcceptedAngle2 && angle < maxAcceptedAngle2)) return true;
            else return false;
        }
    
        private boolean isRangeGood(float range){
            if((range < maxAcceptedRange) && (range > minAcceptedRange)) return true;
            else return false;
        }
    }


    /// Internal IO processing object.
    LidarIO lidarIO;
    /// Internal IO processing thread.
    Thread lidarThread;

    // Cache/shadow variables of internal synchronized variables.
    int lidarUpdatesReceived = 0;
    double scanTimestamp = 0.0;
    int scanCount = 0;
    double avgX = 0.0;
    double slope = 0.0;
    boolean portOK = false;

    public Lidar () {

        // Create the internal LidarIO object:
        lidarIO = new LidarIO();
        
        // Run the internal LidarIO object thread:
        lidarThread = new Thread(lidarIO);
        lidarThread.start(); // Run the LidarIO as background thread.
    }


    public int  getUpdatesReceived() {
        return lidarUpdatesReceived;
    }

    /// Synchronized access to latest timestamp of data.
    public double getScanTimestamp() {
        return scanTimestamp;
    }

    public int getScanCount() {
        return scanCount;
    }

    public double getAvgX() {
        return avgX;
    }

    public double getSlope() {
        return slope;
    }

    public boolean isOK() {
        return portOK;
    }

    @Override
    public void periodic() {
        portOK = lidarIO.isOK();

        if (!portOK){
            return;
        }

        // Access synchronized methods *exactly once* per cycle and cache data in the outer class:
        lidarUpdatesReceived = lidarIO.getUpdatesReceived();
        scanCount = lidarIO.getScanCount();
        avgX = lidarIO.getAvgX();
        slope = lidarIO.getSlope();

        SmartDashboard.putNumber("Lidar/Updates", lidarIO.getUpdatesReceived());
        SmartDashboard.putNumber("Lidar/ScanCount", lidarIO.getScanCount());
        SmartDashboard.putNumber("Lidar/AvgX", lidarIO.getAvgX());
        SmartDashboard.putNumber("Lidar/Slope", lidarIO.getSlope());
        SmartDashboard.putNumber("Lidar/IterationTime", lidarIO.getLastIterationTime());
    }

}