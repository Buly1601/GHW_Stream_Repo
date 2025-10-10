from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import cv2


class ObjectDetection:
    def __init__(self, video_path=None):
        # Load YOLO model
        self.model = YOLO("yolo11s.pt")

        # Initialize DeepSort tracker
        self.tracker = DeepSort(
            max_age=5,
            n_init=2,                    
            max_iou_distance=0.75,
            max_cosine_distance=0.3,
            embedder="mobilenet",
            embedder_gpu=True
        )

        # Parse video
        self._parse_video(video_path)

        # Compute the video
        self.compute_and_predict()


    def _predict_image(self, image):
        """
        Function to predict an image used to debug the model's performance in computer
        """
        # get the results 
        results = self.model(image, verbose=False)[0]
        
        # read img with opencv
        cv_img = cv2.imread(image)

        # Loop over all detections
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            conf = float(box.conf[0])
            cls = int(box.cls[0])

            # Get class name (works if your model has names loaded)
            class_name = self.model.names.get(cls, str(cls))

            # Draw bounding box
            cv2.rectangle(cv_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

            # Label with class name + confidence
            label = f"{class_name} {conf:.2f}"
            cv2.putText(cv_img, label, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Show image
        cv2.imshow("YOLO Prediction Debug", cv_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        

    def _parse_video(self, video_path):
        """
        Function to parse the video from source into a readable object.
        """
        self.video = cv2.VideoCapture(video_path)

        if not self.video.isOpened():
            print("Error parsing the video.")
            self.video = None


    def compute_and_predict(self):
        """
        Computes and predicts based on the video input
        """
        if not self.video:
            print("No video source available.")
            return -1

        while True:
            ok, frame = self.video.read()
            if not ok:
                print("End of video or error reading frame.")
                break

            # YOLO prediction
            results = self.model(frame)[0]

            # Extract detections (x1, y1, x2, y2, confidence, class_id)
            detections = []
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                detections.append(( [float(x1), float(y1), float(x2 - x1), float(y2 - y1)], float(conf), int(cls) ))

                
            # Update DeepSort tracker
            tracks = self.tracker.update_tracks(detections, frame=frame)

            # Draw tracked boxes
            for track in tracks:
                if not track.is_confirmed():
                    continue
                x1, y1, x2, y2 = track.to_ltrb()
                track_id = track.track_id

                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"ID {track_id}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Show frame
            cv2.imshow("YOLO + DeepSORT Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.video.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    detector = ObjectDetection()
    detector._parse_video("images/soccer.mp4")
    detector.compute_and_predict()
