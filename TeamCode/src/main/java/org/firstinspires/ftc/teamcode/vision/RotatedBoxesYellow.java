package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import org.openftc.easyopencv.OpenCvPipeline;


public class RotatedBoxesYellow extends OpenCvPipeline {

	public double cX = 0;
	public double cY = 0;
	public double width = 0;

	public Scalar lowerHSV = new Scalar(9.0, 0.0, 28.0, 0.0);
	public Scalar upperHSV = new Scalar(22.0, 183.0, 255.0, 0.0);
	private Mat hsvMat = new Mat();
	private Mat hsvBinaryMat = new Mat();

	private ArrayList<MatOfPoint> contours = new ArrayList<>();
	private Mat hierarchy = new Mat();

	public int minArea = 6000;
	public int maxArea = 250000;
	private ArrayList<MatOfPoint> contoursByArea = new ArrayList<>();

	private MatOfPoint2f contoursByArea2f = new MatOfPoint2f();
	private ArrayList<RotatedRect> contoursByAreaRotRects = new ArrayList<>();

	public Scalar lineColor = new Scalar(255.0, 0.0, 255.0, 0.0);
	public int lineThickness = 4;

	private Mat inputRotRects = new Mat();

	public List<Point> rectangleCenters = new ArrayList<>();

	// Initialize the DistanceCalculator with known parameters
	private DistanceCalculator distanceCalculator = new DistanceCalculator(3.5, 600.0); // Real width: 3.5 inches, Focal length: 600 pixels

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
		rectangleCenters.clear(); // Clear the list of centers before updating
		for (MatOfPoint points : contoursByArea) {
			contoursByArea2f.release();
			points.convertTo(contoursByArea2f, CvType.CV_32F);

			RotatedRect rect = Imgproc.minAreaRect(contoursByArea2f);

			// Ensure the longer side is always the width
			if (rect.size.width < rect.size.height) {
				// Swap the dimensions to make sure width is the longer side
				Size tempSize = rect.size;
				rect.size = new Size(tempSize.height, tempSize.width);

				// Adjust the angle accordingly
				rect.angle = rect.angle + 90; // Add 90 degrees to align width as the longer side
			}

			// Normalize the angle to the range [-90, 90]
			if (rect.angle > 90) {
				rect.angle -= 180;
			} else if (rect.angle < -90) {
				rect.angle += 180;
			}

			contoursByAreaRotRects.add(rect);

			// Add the center of the rotated rectangle to the list
			if (rect != null) {
				rectangleCenters.add(rect.center);
			}
		}

		input.copyTo(inputRotRects);
		for (RotatedRect rect : contoursByAreaRotRects) {
			if (rect != null) {
				// Draw the rotated rectangle
				Point[] rectPoints = new Point[4];
				rect.points(rectPoints);
				MatOfPoint matOfPoint = new MatOfPoint(rectPoints);
				Imgproc.polylines(inputRotRects, Collections.singletonList(matOfPoint), true, lineColor, lineThickness);

				// Get the center, angle, and size of the rotated rectangle
				Point center = rect.center;
				double angle = Math.toRadians(rect.angle);
				double halfWidth = rect.size.width / 2;

				// Draw the center point as a red circle
				Imgproc.circle(inputRotRects, center, 5, new Scalar(0, 0, 255), -1); // Red circle with radius 5

				// Calculate the line endpoints to represent the angle
				Point end1 = new Point(
						center.x + halfWidth * Math.cos(angle),
						center.y + halfWidth * Math.sin(angle)
				);
				Point end2 = new Point(
						center.x - halfWidth * Math.cos(angle),
						center.y - halfWidth * Math.sin(angle)
				);

				// Draw the line along the width
				Imgproc.line(inputRotRects, end1, end2, new Scalar(0, 255, 0), 2);

				// Calculate x and y offsets
				int xOffset = (int) (center.x - (input.width() / 2));
				int yOffset = (int) (center.y - (input.height() / 2));

				// Display the x and y offsets as text
				String offsetText = String.format("PixelOffset: (%d, %d)", xOffset, yOffset);
				Imgproc.putText(
						inputRotRects,
						offsetText,
						new Point(center.x - 50, center.y + 70), // Offset the text for visibility
						Imgproc.FONT_HERSHEY_SIMPLEX,
						0.5, // Font scale
						new Scalar(255, 255, 255), // Text color
						1 // Thickness
				);

				// Calculate and display the distance
				double perceivedWidth = rect.size.width;
				double distance = distanceCalculator.calculateDistance(perceivedWidth);

				// Display the distance as text
				String distanceText = String.format("Distance: %.2f in", distance);
				Imgproc.putText(
						inputRotRects,
						distanceText,
						new Point(center.x - 50, center.y + 50), // Offset the text for visibility
						Imgproc.FONT_HERSHEY_SIMPLEX,
						0.5, // Font scale
						new Scalar(255, 255, 255), // Text color
						1 // Thickness
				);

				// Optional: Display the angle as text
				String angleText = String.format("Angle: %.2f", rect.angle);
				Imgproc.putText(
						inputRotRects,
						angleText,
						new Point(center.x - 50, center.y - 50), // Offset the text for visibility
						Imgproc.FONT_HERSHEY_SIMPLEX,
						0.5, // Font scale
						new Scalar(255, 255, 255), // Text color
						1 // Thickness
				);
			}
		}

		return inputRotRects;
	}

	// DistanceCalculator class
	public static class DistanceCalculator {
		private double realWidth; // Real-world width of the object (in inches)
		private double focalLength; // Focal length of the camera (in pixels)

		// Constructor to initialize realWidth and focalLength
		public DistanceCalculator(double realWidth, double focalLength) {
			this.realWidth = realWidth;
			this.focalLength = focalLength;
		}

		// Method to calculate distance
		public double calculateDistance(double perceivedWidth) {
			if (perceivedWidth <= 0) {
				throw new IllegalArgumentException("Perceived width must be greater than 0");
			}
			return (realWidth * focalLength) / perceivedWidth;
		}
	}
}
