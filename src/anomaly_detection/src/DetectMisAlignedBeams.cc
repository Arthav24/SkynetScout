/**
 * @brief This header file includes declaration for
          misaligned beams detection.
 * @file DetectMisAlignedBeams.cc
 * @date November 19 2024
 * @version 1.5
 * @author Anirudh
 * @copyright Copyright (c) 2024
 */

#include "DetectMisAlignedBeams.h"

/*constructor misaligned beams*/
scout::MisalignedBeams::MisalignedBeams() {
  status = skynet_interfaces::msg::AnomalyStatus();
  status.name = "Beam Misalignment";
  status.message = "";
  status.level = skynet_interfaces::msg::AnomalyStatus::OK;
}
scout::MisalignedBeams::~MisalignedBeams() {
  /*destructor*/
};

/*process image method*/
skynet_interfaces::msg::AnomalyStatus
scout::MisalignedBeams::processImage(cv::Mat image) {

  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  // Threshold the image to create a binary image
  cv::Mat binary;
  threshold(gray, binary, 100, 255, cv::THRESH_BINARY);

  // Find contours of the beams
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL,
               cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Rect> boundingBoxes;

  for (const auto &contour : contours) {
    cv::Rect box = boundingRect(contour);

    // Filter out small contours (noise)
    if (box.width > 50 && box.height > 20) { // Adjusted for horizontal beams
      boundingBoxes.push_back(box);
      rectangle(image, box, cv::Scalar(0, 255, 0),
                2); // Draw bounding box on the input image
    }
  }

  // Sort bounding boxes by their horizontal (x) position
  std::sort(boundingBoxes.begin(), boundingBoxes.end(),
            [](const cv::Rect &a, const cv::Rect &b) { return a.x < b.x; });

  // Group beams based on horizontal distance
  std::vector<std::vector<cv::Rect>> beamGroups;
  std::vector<cv::Rect> currentGroup;

  const int horizontalThreshold =
      50; // Max horizontal distance to consider the same group

  for (size_t i = 0; i < boundingBoxes.size(); ++i) {
    if (currentGroup.empty() ||
        abs(boundingBoxes[i].x - currentGroup.back().x) < horizontalThreshold) {
      currentGroup.push_back(boundingBoxes[i]);
    } else {
      beamGroups.push_back(currentGroup);
      currentGroup.clear();
      currentGroup.push_back(boundingBoxes[i]);
    }
  }
  if (!currentGroup.empty()) {
    beamGroups.push_back(currentGroup);
  }

  // Analyze vertical pixel continuity within each group
  const int misalignmentThreshold =
      100; // Allowed variation in white pixel width

  for (size_t groupIndex = 0; groupIndex < beamGroups.size(); ++groupIndex) {
    const auto &group = beamGroups[groupIndex];
    bool isMisaligned = false;

    for (const auto &box : group) {
      // Extract the region of interest (ROI) for the current beam
      cv::Mat roi = binary(box);

      // Calculate white pixel width at the bottom
      int bottomRow = roi.rows - 1;
      int whitePixelWidth = countNonZero(roi.row(bottomRow));

      // Check white pixel continuity along the vertical axis
      for (int row = bottomRow - 1; row >= 0; --row) {
        int currentWidth = countNonZero(roi.row(row));
        if (abs(currentWidth - whitePixelWidth) > misalignmentThreshold) {
          isMisaligned = true;
          break;
        }
      }

      if (isMisaligned) {
        //        putText(image, "Misaligned",
        //                cv::Point(box.x, box.y - 10),
        //                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255),
        //                2);
        //        rectangle(image, box, cv::Scalar(0, 0, 255), 2); // Highlight
        //        misaligned box

        status.level = skynet_interfaces::msg::AnomalyStatus::MBEAMS;
        status.message = "Found misaligned beams";
        // TODO need to translate to real world
        status.position.position.x = box.x;
        status.position.position.y = box.y - 10;

      } else {
        //        putText(image, "Aligned",
        //                cv::Point(box.x, box.y - 10),
        //                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0),
        //                2);
        //      }
        status.level = skynet_interfaces::msg::AnomalyStatus::OK;
        status.message = "";
        status.position.position.x = 0;
        status.position.position.y = 0;
      }
    }

    // // Annotate group for clarity
    // string groupLabel = "Group " + to_string(groupIndex + 1);
    // putText(image, groupLabel,
    //         Point(group.front().x, group.front().y - 20),
    //         FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
  }

  // Display the result
  //  cv::imshow("Beam Alignment Detection", image);
  //  cv::waitKey(0);

  return status;
}
