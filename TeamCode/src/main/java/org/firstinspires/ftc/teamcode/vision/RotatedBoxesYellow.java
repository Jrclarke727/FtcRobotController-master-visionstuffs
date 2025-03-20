package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import org.openftc.easyopencv.OpenCvPipeline;



@Config
public class RotatedBoxesYellow extends OpenCvPipeline {

	public static int xTarget = 240;
	// Default to center of a 640px wide image
	public static int yTarget = 320; // User-defined target Y

	public double cX = 0;
	public double cY = 0;
	public double width = 0;

	public Scalar lowerHSV = new Scalar(16.0, 148.0, 35.0, 0.0);
	public Scalar upperHSV = new Scalar(44.0, 255.0, 255.0, 0.0);
	private Mat hsvMat = new Mat();
	private Mat hsvBinaryMat = new Mat();

	private ArrayList<MatOfPoint> contours = new ArrayList<>();
	private Mat hierarchy = new Mat();

	public int minArea = 6000;
	public int maxArea = 250000;
	private ArrayList<MatOfPoint> contoursByArea = new ArrayList<>();

	public MatOfPoint2f contoursByArea2f = new MatOfPoint2f();
	public ArrayList<RotatedRect> contoursByAreaRotRects = new ArrayList<>();

	public Scalar lineColor = new Scalar(255.0, 0.0, 255.0, 0.0);
	public int lineThickness = 4;

	private Mat inputRotRects = new Mat();

	public List<Point> rectangleCenters = new ArrayList<>();

	private DistanceCalculator distanceCalculator = new DistanceCalculator(3.5, 600.0);

	@Override
	public Mat processFrame(Mat input) {
		Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
		Core.inRange(hsvMat, lowerHSV, upperHSV, hsvBinaryMat);

		contours.clear();
		hierarchy.release();
		Imgproc.findContours(hsvBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		contoursByArea.clear();
		for (MatOfPoint contour : contours) {
			double area = Imgproc.contourArea(contour);
			if ((area >= minArea) && (area <= maxArea)) {
				contoursByArea.add(contour);
			}
		}

		contoursByAreaRotRects.clear();
		rectangleCenters.clear();
		for (MatOfPoint points : contoursByArea) {
			contoursByArea2f.release();
			points.convertTo(contoursByArea2f, CvType.CV_32F);

			RotatedRect rect = Imgproc.minAreaRect(contoursByArea2f);

			if (rect.size.width < rect.size.height) {
				Size tempSize = rect.size;
				rect.size = new Size(tempSize.height, tempSize.width);
				rect.angle = rect.angle + 90;
			}

			if (rect.angle > 90) {
				rect.angle -= 180;
			} else if (rect.angle < -90) {
				rect.angle += 180;
			}

			contoursByAreaRotRects.add(rect);

			if (rect != null) {
				rectangleCenters.add(rect.center);
			}
		}

		input.copyTo(inputRotRects);
		TelemetryPacket packet = new TelemetryPacket(); // FTC Dashboard telemetry

		for (RotatedRect rect : contoursByAreaRotRects) {
			if (rect != null) {
				Point[] rectPoints = new Point[4];
				rect.points(rectPoints);
				MatOfPoint matOfPoint = new MatOfPoint(rectPoints);
				Imgproc.polylines(inputRotRects, Collections.singletonList(matOfPoint), true, lineColor, lineThickness);

				Point center = rect.center;
				double angle = Math.toRadians(rect.angle);
				double halfWidth = rect.size.width / 2;

				Imgproc.circle(inputRotRects, center, 5, new Scalar(0, 0, 255), -1);

				Point end1 = new Point(center.x + halfWidth * Math.cos(angle), center.y + halfWidth * Math.sin(angle));
				Point end2 = new Point(center.x - halfWidth * Math.cos(angle), center.y - halfWidth * Math.sin(angle));
				Imgproc.line(inputRotRects, end1, end2, new Scalar(0, 255, 0), 2);

				int xOffset = (int) (center.x - xTarget);
				int yOffset = (int) (center.y - yTarget);

				String offsetText = String.format("PixelOffset: (%d, %d)", xOffset, yOffset);
				Imgproc.putText(inputRotRects, offsetText, new Point(center.x - 50, center.y + 70),
						Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);

				double perceivedWidth = rect.size.width;
				double distance = distanceCalculator.calculateDistance(perceivedWidth);

				String distanceText = String.format("Distance: %.2f in", distance);
				Imgproc.putText(inputRotRects, distanceText, new Point(center.x - 50, center.y + 50),
						Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);

				String angleText = String.format("Angle: %.2f", rect.angle);
				Imgproc.putText(inputRotRects, angleText, new Point(center.x - 50, center.y - 50),
						Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);

				// Send values to FTC Dashboard
				packet.put("xOffset", xOffset);
				packet.put("yOffset", yOffset);
				packet.put("Distance (in)", distance);
				packet.put("Angle (deg)", rect.angle);
			}
		}

		return inputRotRects;
	}

	public static class DistanceCalculator {
		private double realWidth;
		private double focalLength;

		public DistanceCalculator(double realWidth, double focalLength) {
			this.realWidth = realWidth;
			this.focalLength = focalLength;
		}

		public double calculateDistance(double perceivedWidth) {
			if (perceivedWidth <= 0) {
				throw new IllegalArgumentException("Perceived width must be greater than 0");
			}
			return (realWidth * focalLength) / perceivedWidth;
		}
	}
}
