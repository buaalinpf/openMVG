// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/types.hpp"
#include "software/SfM/SfMPlyHelper.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include <unordered_set>
#include "openMVG/sfm/sfm_data_colorization.hpp"

using namespace openMVG;
using namespace openMVG::sfm;

/// Export camera poses positions as a Vec3 vector
void GetRepeatedCameras(const SfM_Data& sfm_data,
	const SfM_Data& sfm_data1,
	const SfM_Data& sfm_data2,
	const SfM_Data& sfm_data3,
	const SfM_Data& sfm_data4,
	const SfM_Data& sfm_data5,
	const SfM_Data& sfm_data6,
	const SfM_Data& sfm_data7,
	const SfM_Data& sfm_data8,
	std::unordered_set<int>& isRepeated ) 
{
	std::unordered_set<int> isPrinted;
	for (const auto& view : sfm_data.GetViews()) {
		if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get())) {
			auto pose_id = view.second->id_pose;
			if (sfm_data1.poses.find(pose_id) != sfm_data1.poses.end())isPrinted.insert(pose_id);
			if (sfm_data2.poses.find(pose_id) != sfm_data2.poses.end()) {
				if (isPrinted.count(pose_id) == 1) {
					isRepeated.insert(pose_id);
					continue;
				}
				else isPrinted.insert(pose_id);
			}
			if (sfm_data3.poses.find(pose_id) != sfm_data3.poses.end()) {
				if (isPrinted.count(pose_id) == 1) {
					isRepeated.insert(pose_id);
					continue;
				}
				else isPrinted.insert(pose_id);
			}
			if (sfm_data4.poses.find(pose_id) != sfm_data4.poses.end()) {
				if (isPrinted.count(pose_id) == 1) {
					isRepeated.insert(pose_id);
					continue;
				}
				else isPrinted.insert(pose_id);
			}
			if (sfm_data5.poses.find(pose_id) != sfm_data5.poses.end()) {
				if (isPrinted.count(pose_id) == 1) {
					isRepeated.insert(pose_id);
					continue;
				}
				else isPrinted.insert(pose_id);
			}
			if (sfm_data6.poses.find(pose_id) != sfm_data6.poses.end()) {
				if (isPrinted.count(pose_id) == 1) {
					isRepeated.insert(pose_id);
					continue;
				}
				else isPrinted.insert(pose_id);
			}
			if (sfm_data7.poses.find(pose_id) != sfm_data7.poses.end()) {
				if (isPrinted.count(pose_id) == 1) {
					isRepeated.insert(pose_id);
					continue;
				}
				else isPrinted.insert(pose_id);
			}
			if (sfm_data8.poses.find(pose_id) != sfm_data8.poses.end()) {
				if (isPrinted.count(pose_id) == 1) {
					isRepeated.insert(pose_id);
					continue;
				}
				else isPrinted.insert(pose_id);
			}
		}
	}
}

void GetCameraPositions(const SfM_Data & sfm_data, 
	const SfM_Data& sfm_data_cmp,
	std::vector<Vec3> & vec_camPosition,
	std::unordered_set<int>& isRepeated, 
	std::vector<Vec3>& black_camPosition)
{
  for (const auto & view : sfm_data.GetViews())
  {
    if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get()))
    {
	  auto pose_id = view.second->id_pose;
	  if (isRepeated.count(pose_id) == 1) {
		  const geometry::Pose3 pose = sfm_data.GetPoseOrDie(view.second.get());
		  black_camPosition.push_back(pose.center());
		  continue;
	  }
	  if (sfm_data_cmp.poses.find(pose_id) != sfm_data_cmp.poses.end()) {
		  const geometry::Pose3 pose = sfm_data.GetPoseOrDie(view.second.get());
		  vec_camPosition.push_back(pose.center());
	  }
    }
  }
}

// Convert from a SfM_Data format to another
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string
    sSfM_Data_Filename_In,
    sOutputPLY_Out;

  cmd.add(make_option('i', sSfM_Data_Filename_In, "input_file"));
  cmd.add(make_option('o', sOutputPLY_Out, "output_file"));

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch (const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
        << "[-i|--input_file] path to the input SfM_Data scene\n"
        << "[-o|--output_file] path to the output PLY file\n"
        << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  if (sOutputPLY_Out.empty())
  {
    std::cerr << std::endl
      << "No output PLY filename specified." << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data sfm_data; //F:\\sfm_dataset\\test_18\\robust.bin
  if (!Load(sfm_data, sSfM_Data_Filename_In, ESfM_Data(ALL)))
  {
    std::cerr << std::endl
      << "The input SfM_Data file \"" << sSfM_Data_Filename_In << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }
  SfM_Data sfm_data1;
  if (!Load(sfm_data1, "F:\\sfm_dataset\\tjgy10_0seq\\sfm_data.bin", ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS))) {
	  std::cerr << std::endl
		  << "The input SfM_Data file \"" << "sfm_data1" << "\" cannot be read." << std::endl;
	  return EXIT_FAILURE;
  }

  SfM_Data sfm_data2;
  if (!Load(sfm_data2, "F:\\sfm_dataset\\tjgy10_1seq\\sfm_data.bin", ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS))) {
	  std::cerr << std::endl
		  << "The input SfM_Data file \"" << "sfm_data2" << "\" cannot be read." << std::endl;
	  return EXIT_FAILURE;
  }

  SfM_Data sfm_data3;
  if (!Load(sfm_data3, "F:\\sfm_dataset\\tjgy10_2seq\\sfm_data.bin", ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS))) {
	  std::cerr << std::endl
		  << "The input SfM_Data file \"" << "sfm_data3" << "\" cannot be read." << std::endl;
	  return EXIT_FAILURE;
  }

  SfM_Data sfm_data4;
  if (!Load(sfm_data4, "F:\\sfm_dataset\\tjgy10_3seq\\sfm_data.bin", ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS))) {
	  std::cerr << std::endl
		  << "The input SfM_Data file \"" << "sfm_data4" << "\" cannot be read." << std::endl;
	  return EXIT_FAILURE;
  }

  SfM_Data sfm_data5;
  if (!Load(sfm_data5, "F:\\sfm_dataset\\tjgy10_4seq\\sfm_data.bin", ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS))) {
	  std::cerr << std::endl
		  << "The input SfM_Data file \"" << "sfm_data5" << "\" cannot be read." << std::endl;
	  return EXIT_FAILURE;
  }

  SfM_Data sfm_data6;
  if (!Load(sfm_data6, "F:\\sfm_dataset\\tjgy10_5seq\\sfm_data.bin", ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS))) {
	  std::cerr << std::endl
		  << "The input SfM_Data file \"" << "sfm_data6" << "\" cannot be read." << std::endl;
	  return EXIT_FAILURE;
  }

  SfM_Data sfm_data7;
  if (!Load(sfm_data7, "F:\\sfm_dataset\\tjgy10_6seq\\sfm_data.bin", ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS))) {
	  std::cerr << std::endl
		  << "The input SfM_Data file \"" << "sfm_data7" << "\" cannot be read." << std::endl;
	  return EXIT_FAILURE;
  }

  SfM_Data sfm_data8;
  if (!Load(sfm_data8, "F:\\sfm_dataset\\tjgy10_7seq\\sfm_data.bin", ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS))) {
	  std::cerr << std::endl
		  << "The input SfM_Data file \"" << "sfm_data8" << "\" cannot be read." << std::endl;
	  return EXIT_FAILURE;
  }



  std::unordered_set<int> isRepeated;
  // Compute the scene structure color
  std::vector<Vec3> vec_camPosition1, vec_camPosition2, vec_camPosition3, vec_camPosition4, vec_camPosition5, vec_camPosition6, vec_camPosition7, vec_camPosition8;
  std::vector<Vec3> vec_3dPoints, vec_tracksColor, black_camPosition;
  if (ColorizeTracks(sfm_data, vec_3dPoints, vec_tracksColor))
  {
	  GetRepeatedCameras(sfm_data, sfm_data1, sfm_data2, sfm_data3, sfm_data4, sfm_data5, sfm_data6, sfm_data7, sfm_data8, isRepeated);
    GetCameraPositions(sfm_data, sfm_data1, vec_camPosition1, isRepeated, black_camPosition);
	GetCameraPositions(sfm_data, sfm_data2, vec_camPosition2, isRepeated, black_camPosition);
	GetCameraPositions(sfm_data, sfm_data3, vec_camPosition3, isRepeated, black_camPosition);
	GetCameraPositions(sfm_data, sfm_data4, vec_camPosition4, isRepeated, black_camPosition);
	GetCameraPositions(sfm_data, sfm_data5, vec_camPosition5, isRepeated, black_camPosition);
	GetCameraPositions(sfm_data, sfm_data6, vec_camPosition6, isRepeated, black_camPosition);
	GetCameraPositions(sfm_data, sfm_data7, vec_camPosition7, isRepeated, black_camPosition);
	GetCameraPositions(sfm_data, sfm_data8, vec_camPosition8, isRepeated, black_camPosition);
    // Export the SfM_Data scene in the expected format
    if (plyHelper::exportToPly(vec_3dPoints, black_camPosition, vec_camPosition1, vec_camPosition2, vec_camPosition3, vec_camPosition4, vec_camPosition5, vec_camPosition6, vec_camPosition7, vec_camPosition8, sOutputPLY_Out, &vec_tracksColor))
    {
      return EXIT_SUCCESS;
    }
  }

  return EXIT_FAILURE;
}
