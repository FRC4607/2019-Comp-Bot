package frc.robot.lib.drivers;

import org.junit.*;
import static org.junit.Assert.assertEquals;
import frc.robot.lib.drivers.Limelight;
import frc.robot.lib.drivers.Limelight.snapshot;
import frc.robot.lib.drivers.Limelight.stream;
import frc.robot.lib.drivers.Limelight.pipeline;
import frc.robot.lib.drivers.Limelight.camMode;
import frc.robot.lib.drivers.Limelight.ledMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightTest {

  Limelight mLimelight1 = new Limelight();
  Limelight mLimelight2 = new Limelight("limelight2");
  NetworkTable mTable1 = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTable mTable2 = NetworkTableInstance.getDefault().getTable("limelight2");

  public void ledModeCheck(ledMode lmode, double checkValue) {
		mLimelight1.setLedMode(lmode);
		mLimelight2.setLedMode(lmode);
		assertEquals(checkValue, mLimelight1.getLedMode(), 3.0);
		assertEquals(checkValue, mLimelight2.getLedMode(), 3.0);
  }

  public void camModeCheck(camMode cmode, double checkValue) {
		mLimelight1.setCamMode(cmode);
		mLimelight2.setCamMode(cmode);
		assertEquals(checkValue, mLimelight1.getCamMode(), 0.0);
		assertEquals(checkValue, mLimelight2.getCamMode(), 0.0);
  }

  public void pipelineCheck(pipeline pl, double checkValue) {
		mLimelight1.setPipeline(pl);
		mLimelight2.setPipeline(pl);
		assertEquals(checkValue, mLimelight1.getPipeline(), 0.0);
		assertEquals(checkValue, mLimelight2.getPipeline(), 0.0);
  }

  public void streamCheck(stream stm, double checkValue) {
		mLimelight1.setStream(stm);
		mLimelight2.setStream(stm);
		assertEquals(checkValue, mLimelight1.getStream(), 0.0);
		assertEquals(checkValue, mLimelight2.getStream(), 0.0);
  }

  public void snapshotCheck(snapshot sshot, double checkValue) {
		mLimelight1.setSnapshot(sshot);
		mLimelight2.setSnapshot(sshot);
		assertEquals(checkValue, mLimelight1.getSnapshot(), 0.0);
		assertEquals(checkValue, mLimelight2.getSnapshot(), 0.0);
  }

  @Test
  public void testLedMode() {
		ledModeCheck(ledMode.kCurrentPipeline, 0.0);
		ledModeCheck(ledMode.kOff, 1.0);
		ledModeCheck(ledMode.kBlink, 2.0);
		ledModeCheck(ledMode.kOff, 3.0);		
  }
	
  @Test
  public void testCamMode() {
		camModeCheck(camMode.kVision, 0.0);
		camModeCheck(camMode.kDriver, 1.0);
  }

  @Test
  public void testPipeline() {
		pipelineCheck(pipeline.k0, 0.0);
		pipelineCheck(pipeline.k1, 1.0);
		pipelineCheck(pipeline.k2, 2.0);
		pipelineCheck(pipeline.k3, 3.0);
		pipelineCheck(pipeline.k4, 4.0);
		pipelineCheck(pipeline.k5, 5.0);
		pipelineCheck(pipeline.k6, 6.0);
		pipelineCheck(pipeline.k7, 7.0);
		pipelineCheck(pipeline.k8, 8.0);
		pipelineCheck(pipeline.k9, 9.0);
  }

  @Test
  public void testStream() {
		streamCheck(stream.kStandard, 0.0);
		streamCheck(stream.kPipMain, 1.0);
		streamCheck(stream.kPipSecondary, 2.0);
  }

  @Test
  public void testSnapshot() {
		snapshotCheck(snapshot.stop, 0.0);
		snapshotCheck(snapshot.start, 1.0);
  }

  @Test
  public void testFoundTarget() {
		mTable1.getEntry("tv").setValue(0.0);
		mTable2.getEntry("tv").setValue(0.0);
		assertEquals(false, mLimelight1.foundTarget());
		assertEquals(false, mLimelight2.foundTarget());
		mTable1.getEntry("tv").setValue(1.0);
		mTable2.getEntry("tv").setValue(1.0);
		assertEquals(true, mLimelight1.foundTarget());
		assertEquals(true, mLimelight2.foundTarget());
  }

  @Test
  public void testHorizontalToTargetDeg() {
	  mTable1.getEntry("tx").setValue(15.2);
	  mTable2.getEntry("tx").setValue(15.2);
	  assertEquals(15.2, mLimelight1.horizontalToTargetDeg(), 0.0);
	  assertEquals(15.2, mLimelight2.horizontalToTargetDeg(), 0.0);
	  mTable1.getEntry("tx").setValue(-15.2);
	  mTable2.getEntry("tx").setValue(-15.2);
	  assertEquals(-15.2, mLimelight1.horizontalToTargetDeg(), 0.0);
    assertEquals(-15.2, mLimelight2.horizontalToTargetDeg(), 0.0);
  }

  @Test
  public void testVerticalToTargetDeg() {
	  mTable1.getEntry("ty").setValue(15.2);
	  mTable2.getEntry("ty").setValue(15.2);
	  assertEquals(15.2, mLimelight1.verticalToTargetDeg(), 0.0);
	  assertEquals(15.2, mLimelight2.verticalToTargetDeg(), 0.0);
	  mTable1.getEntry("ty").setValue(-15.2);
	  mTable2.getEntry("ty").setValue(-15.2);
	  assertEquals(-15.2, mLimelight1.verticalToTargetDeg(), 0.0);
	  assertEquals(-15.2, mLimelight2.verticalToTargetDeg(), 0.0);
  }

  @Test
  public void testTargetAreaPercent() {
	  mTable1.getEntry("ta").setValue(15.2);
	  mTable2.getEntry("ta").setValue(15.2);
	  assertEquals(15.2, mLimelight1.targetAreaPercent(), 0.0);
	  assertEquals(15.2, mLimelight2.targetAreaPercent(), 0.0);
  }

  @Test
  public void testSkewDeg() {
	  mTable1.getEntry("ts").setValue(15.2);
	  mTable2.getEntry("ts").setValue(15.2);
	  assertEquals(15.2, mLimelight1.skewDeg(), 0.0);
	  assertEquals(15.2, mLimelight2.skewDeg(), 0.0);
  }

  @Test
  public void testLatencyMs() {
	  mTable1.getEntry("tl").setValue(15.2);
	  mTable2.getEntry("tl").setValue(15.2);
	  assertEquals(15.2, mLimelight1.latencyMs(), 0.0);
	  assertEquals(15.2, mLimelight2.latencyMs(), 0.0);
  }

  @Test
  public void testShortSidePixels() {
	  mTable1.getEntry("tshort").setValue(15.2);
	  mTable2.getEntry("tshort").setValue(15.2);
	  assertEquals(15.2, mLimelight1.shortSidePixels(), 0.0);
	  assertEquals(15.2, mLimelight2.shortSidePixels(), 0.0);
  }

  @Test
  public void testLongSidePixels() {
	  mTable1.getEntry("tlong").setValue(15.2);
	  mTable2.getEntry("tlong").setValue(15.2);
	  assertEquals(15.2, mLimelight1.longSidePixels(), 0.0);
	  assertEquals(15.2, mLimelight2.longSidePixels(), 0.0);
  }

  @Test
  public void testHorizontalPixels() {
	  mTable1.getEntry("thor").setValue(15.2);
	  mTable2.getEntry("thor").setValue(15.2);
	  assertEquals(15.2, mLimelight1.horizontalPixels(), 0.0);
	  assertEquals(15.2, mLimelight2.horizontalPixels(), 0.0);
  }

  @Test
  public void testVerticalPixels() {
	  mTable1.getEntry("tvert").setValue(15.2);
	  mTable2.getEntry("tvert").setValue(15.2);
	  assertEquals(15.2, mLimelight1.verticalPixels(), 0.0);
	  assertEquals(15.2, mLimelight2.verticalPixels(), 0.0);
  }
 
  @Test
  public void testGetTruePipeline() {
	  mTable1.getEntry("getpipe").setValue(15.2);
	  mTable2.getEntry("getpipe").setValue(15.2);
	  assertEquals(15.2, mLimelight1.getTruePipeline(), 0.0);
	  assertEquals(15.2, mLimelight2.getTruePipeline(), 0.0);
  }

	@Test
	public void testGetCornersX() {
		double[] setValue1 = {1.0, 2.0, 3.0, 4.0};
		double[] setValue2 = {5.0, 6.0, 7.0, 8.0};
		  
		mTable1.getEntry("tcornx").setValue(setValue1);
		double[] retValue = mLimelight1.getCornersX();
		assertEquals(setValue1[0], retValue[0], 0.0);
		assertEquals(setValue1[1], retValue[1], 0.0);
		assertEquals(setValue1[2], retValue[2], 0.0);
		assertEquals(setValue1[3], retValue[3], 0.0);
		
		mTable2.getEntry("tcornx").setValue(setValue2);
		retValue = mLimelight2.getCornersX();
		assertEquals(setValue2[0], retValue[0], 0.0);
		assertEquals(setValue2[1], retValue[1], 0.0);
		assertEquals(setValue2[2], retValue[2], 0.0);
		assertEquals(setValue2[3], retValue[3], 0.0);
	}

	@Test
	public void testGetCornersY() {
		double[] setValue1 = {9.0, 10.0, 11.0, 12.0};
		double[] setValue2 = {13.0, 14.0, 15.0, 16.0};
		mTable1.getEntry("tcorny").setValue(setValue1);
		double[] retValue = mLimelight1.getCornersY();
		assertEquals(setValue1[0], retValue[0], 0.0);
		assertEquals(setValue1[1], retValue[1], 0.0);
		assertEquals(setValue1[2], retValue[2], 0.0);
		assertEquals(setValue1[3], retValue[3], 0.0);
		mTable2.getEntry("tcorny").setValue(setValue2);
		retValue = mLimelight2.getCornersY();
		assertEquals(setValue2[0], retValue[0], 0.0);
		assertEquals(setValue2[1], retValue[1], 0.0);
		assertEquals(setValue2[2], retValue[2], 0.0);
		assertEquals(setValue2[3], retValue[3], 0.0);
	}

}

