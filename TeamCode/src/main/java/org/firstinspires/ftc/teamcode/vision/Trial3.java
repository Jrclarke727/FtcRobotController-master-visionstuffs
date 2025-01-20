import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import org.openftc.easyopencv.OpenCvPipeline;

public class

Trial3 extends OpenCvPipeline {

    public Scalar lowerHSV = new Scalar(9.0, 0.0, 0.0, 0.0);
    public Scalar upperHSV = new Scalar(25.0, 192.0, 255.0, 0.0);
    private Mat hsvMat = new Mat();
    private Mat hsvBinaryMat = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    public int minArea = 7000;
    public int maxArea = 250000;
    private ArrayList<MatOfPoint> contoursByArea = new ArrayList<>();

    private ArrayList<Rect> contoursByAreaRects = new ArrayList<>();

    public Scalar lineColor = new Scalar(0.0, 255.0, 0.0, 0.0);
    public int lineThickness = 3;

    private Mat inputRects = new Mat();

    private ArrayList<Target> targets = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lowerHSV, upperHSV, hsvBinaryMat);

        contours.clear();
        hierarchy.release();
        Imgproc.findContours(hsvBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contoursByArea.clear();
        for(MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if((area >= minArea) && (area <= maxArea)) {
                contoursByArea.add(contour);
            }
        }

        contoursByAreaRects.clear();
        for(MatOfPoint points : contoursByArea) {
            contoursByAreaRects.add(Imgproc.boundingRect(points));
        }

        input.copyTo(inputRects);
        for(Rect rect : contoursByAreaRects) {
            Imgproc.rectangle(inputRects, rect, lineColor, lineThickness);
        }

        clearTargets();

        addTargets("Sample", contoursByAreaRects);

        return inputRects;
    }

    private synchronized void addTarget(String label, Rect rect) {
        targets.add(new Target(label, rect));
    }

    private synchronized void addTargets(String label, ArrayList<Rect> rects) {
        for(Rect rect : rects) {
            addTarget(label, rect);
        }
    }

    private synchronized void clearTargets() {
        targets.clear();
    }

    public synchronized Target[] getTargets() {
        Target[] array = new Target[targets.size()];

        if(targets.isEmpty()) {
            return array;
        }

        for(int i = 0 ; i < targets.size() - 1 ; i++) {
            array[i] = ((Target) (targets.get(i)));
        }

        return array;
    }

    public synchronized ArrayList<Target> getTargetsWithLabel(String label) {
        ArrayList<Target> targetsWithLabel = new ArrayList<>();

        for(Target target : targets) {
            if(target.label.equals(label)) {
                targetsWithLabel.add(target);
            }
        }

        return targetsWithLabel;
    }

    public class Target {
        public final String label;
        public final Rect rect;

        protected Target(String label, Rect rect) {
            this.label = label;
            this.rect = rect;
        }
    }
}


