// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013, 2014 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"
#include "openMVG/sfm/pipelines/global/GlobalSfM_rotation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/GlobalSfM_translation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/sfm/sfm_data_triangulation.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::features;

double scale_t(Poses test_pose1, Poses test_pose2, std::vector<int> repeated_view, int repeat_view1, int repeat_view2) {
	double scale_T;
	Mat3 R_spec;  //sfmdata2
	Vec3 T_spec;
	Vec3 scale_part2_t;
	Mat3 scale_part2_r;
	Mat3 R_part1; //sfmdata1
	Vec3 T_part1;
	Vec3 C_part1;
	Vec3 scale_part1_t;
	Mat3 scale_part1_r;
	for (auto it_test1 = test_pose1.begin(); it_test1 != test_pose1.end(); it_test1++) {
		if (it_test1->first == repeat_view1) {
			R_part1 = it_test1->second.rotation();
			T_part1 = it_test1->second.translation();
			C_part1 = it_test1->second.center();
		}

		if (it_test1->first == repeat_view2) {
			scale_part1_t = it_test1->second.translation();
			scale_part1_r = it_test1->second.rotation();
		}
	}
	Vec3 ssss1 = scale_part1_t - scale_part1_r * R_part1.transpose() * T_part1;

	for (auto it_test2 = test_pose2.begin(); it_test2 != test_pose2.end(); it_test2++) {
		if (it_test2->first == repeat_view1) {
			R_spec = it_test2->second.rotation();
			T_spec = it_test2->second.translation();
		}
		if (it_test2->first == repeat_view2) {
			scale_part2_t = it_test2->second.translation();
			scale_part2_r = it_test2->second.rotation();
		}
	}
	Vec3 ssss2 = scale_part2_t - scale_part2_r * R_spec.transpose() * T_spec;

	scale_T = sqrt(ssss1.dot(ssss1)) / sqrt(ssss2.dot(ssss2));
	return scale_T;
}

bool merge2data(SfM_Data& sfm_data1, SfM_Data& sfm_data2) {
	Poses test_pose1 = sfm_data1.poses;
	Poses test_pose2 = sfm_data2.poses;
	//transform R t
	std::vector<int> repeated_view;
	for (auto it_test1 = test_pose1.begin(); it_test1 != test_pose1.end(); it_test1++) {
		for (auto it_test2 = test_pose2.begin(); it_test2 != test_pose2.end(); it_test2++) {
			if (it_test1->first == it_test2->first) {
				repeated_view.push_back(it_test1->first);
				//std::cout << it_test1->first << std::endl;
			}
		}
	}
	if (repeated_view.size() < 2)return false;
	Mat3 R_spec;  //sfmdata2
	Vec3 T_spec;
	Vec3 scale_part2_t;
	Mat3 scale_part2_r;
	Mat3 R_part1; //sfmdata1
	Vec3 T_part1;
	Vec3 C_part1;
	Vec3 scale_part1_t;
	Mat3 scale_part1_r;
	for (auto it_test1 = test_pose1.begin(); it_test1 != test_pose1.end(); it_test1++) {
		if (it_test1->first == repeated_view[0]) {
			R_part1 = it_test1->second.rotation();
			T_part1 = it_test1->second.translation();
			C_part1 = it_test1->second.center();
		}
	}
	
	for (auto it_test2 = test_pose2.begin(); it_test2 != test_pose2.end(); it_test2++) {
		if (it_test2->first == repeated_view[0]) {
			R_spec = it_test2->second.rotation();
			T_spec = it_test2->second.translation();
		}
	}
	std::vector<double> scalet_average;
	for (int i = 1; i < repeated_view.size(); i++) {
		scalet_average.push_back(scale_t(test_pose1, test_pose2, repeated_view, repeated_view[0], repeated_view[i]));
	}
	double scalet_full = 0;
	for (int i = 0; i < scalet_average.size(); i++) {
		scalet_full = scalet_full + scalet_average[i];
	}
	double scale_T = scalet_full/ scalet_average.size(); //average
	std::cout << scale_T << std::endl;

	for (auto it_test2 = sfm_data2.poses.begin(); it_test2 != sfm_data2.poses.end(); it_test2++) {
		if (it_test2->first == repeated_view[0])continue;
		else
		{
			Mat3 r_rel = it_test2->second.rotation_ * R_spec.transpose(); //correct
			Vec3 t_rel = it_test2->second.translation() - r_rel * T_spec;
			it_test2->second.rotation_ = r_rel * R_part1; //new_R  correct

			it_test2->second.center_ = -((it_test2->second.rotation_.transpose()) * (r_rel * T_part1 + scale_T * t_rel)); //t
		}
	}

	test_pose2[repeated_view[0]].rotation_ = R_part1;
	test_pose2[repeated_view[0]].center_ = C_part1;



	//----------------------
	Views data2_views = sfm_data2.views;
	Poses data2_poses = sfm_data2.poses;
	Intrinsics data2_intrinsics = sfm_data2.intrinsics;
	Landmarks data2_structure = sfm_data2.structure;
	Landmarks data2_control_points = sfm_data2.control_points;
	for (auto it = data2_views.begin(); it != data2_views.end(); it++) {
		sfm_data1.views.insert(*it);
	}
	for (auto it = data2_poses.begin(); it != data2_poses.end(); it++) {
		sfm_data1.poses.insert(*it);
	}
	for (auto it = data2_intrinsics.begin(); it != data2_intrinsics.end(); it++) {
		sfm_data1.intrinsics.insert(*it);
	}
	return true;
}

int main(int argc, char **argv)
{
  using namespace std;
  std::cout << std::endl
    << "-----------------------------------------------------------\n"
    << "Global Structure from Motion:\n"
    << "-----------------------------------------------------------\n"
    << "Open Source implementation of the paper:\n"
    << "\"Global Fusion of Relative Motions for "
    << "Robust, Accurate and Scalable Structure from Motion.\"\n"
    << "Pierre Moulon, Pascal Monasse and Renaud Marlet. "
    << " ICCV 2013." << std::endl
    << "------------------------------------------------------------"
    << std::endl;


  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string sMatchesDir, sMatchFilename;
  std::string sOutDir = "";
  int iRotationAveragingMethod = int (ROTATION_AVERAGING_L2);
  int iTranslationAveragingMethod = int (TRANSLATION_AVERAGING_SOFTL1);
  std::string sIntrinsic_refinement_options = "ADJUST_ALL";
  bool b_use_motion_priors = false;

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('m', sMatchesDir, "matchdir") );
  cmd.add( make_option('M', sMatchFilename, "match_file") );
  cmd.add( make_option('o', sOutDir, "outdir") );
  cmd.add( make_option('r', iRotationAveragingMethod, "rotationAveraging") );
  cmd.add( make_option('t', iTranslationAveragingMethod, "translationAveraging") );
  cmd.add( make_option('f', sIntrinsic_refinement_options, "refineIntrinsics") );
  cmd.add( make_switch('P', "prior_usage") );

  try {
    if (argc == 1) throw std::string("Invalid parameter.");
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--input_file] path to a SfM_Data scene\n"
    << "[-m|--matchdir] path to the matches that corresponds to the provided SfM_Data scene\n"
    << "[-o|--outdir] path where the output data will be stored\n"
    << "\n[Optional]\n"
    << "[-r|--rotationAveraging]\n"
      << "\t 1 -> L1 minimization\n"
      << "\t 2 -> L2 minimization (default)\n"
    << "[-t|--translationAveraging]:\n"
      << "\t 1 -> L1 minimization\n"
      << "\t 2 -> L2 minimization of sum of squared Chordal distances\n"
      << "\t 3 -> SoftL1 minimization (default)\n"
    << "[-f|--refineIntrinsics] Intrinsic parameters refinement option\n"
      << "\t ADJUST_ALL -> refine all existing parameters (default) \n"
      << "\t NONE -> intrinsic parameters are held as constant\n"
      << "\t ADJUST_FOCAL_LENGTH -> refine only the focal length\n"
      << "\t ADJUST_PRINCIPAL_POINT -> refine only the principal point position\n"
      << "\t ADJUST_DISTORTION -> refine only the distortion coefficient(s) (if any)\n"
      << "\t -> NOTE: options can be combined thanks to '|'\n"
      << "\t ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT\n"
      <<      "\t\t-> refine the focal length & the principal point position\n"
      << "\t ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION\n"
      <<      "\t\t-> refine the focal length & the distortion coefficient(s) (if any)\n"
      << "\t ADJUST_PRINCIPAL_POINT|ADJUST_DISTORTION\n"
      <<      "\t\t-> refine the principal point position & the distortion coefficient(s) (if any)\n"
    << "[-P|--prior_usage] Enable usage of motion priors (i.e GPS positions)\n"
    << "[-M|--match_file] path to the match file to use.\n"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
      iRotationAveragingMethod > ROTATION_AVERAGING_L2 )  {
    std::cerr << "\n Rotation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
    cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);
  if (intrinsic_refinement_options == static_cast<cameras::Intrinsic_Parameter_Type>(0) )
  {
    std::cerr << "Invalid input for Bundle Adjusment Intrinsic parameter refinement option" << std::endl;
    return EXIT_FAILURE;
  }

  if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
      iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )  {
    std::cerr << "\n Translation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  SfM_Data sfm_data;
  if (!Load(sfm_data, "F:\\sfm_dataset\\test_18\\robust.bin", ESfM_Data(ALL))) {
	  std::cerr << std::endl
		  << "The input SfM_Data file \"" << "sfm_data1" << "\" cannot be read." << std::endl;
	  return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data sfm_data1;
  if (!Load(sfm_data1, "F:\\sfm_dataset\\tjgy10_0seq\\sfm_data.bin", ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< "sfm_data1" << "\" cannot be read." << std::endl;
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
  
  
  
  

	std::cout << "...Export SfM_Data to disk." << std::endl;

	Save(sfm_data, sfm_data1, sfm_data2, sfm_data3, sfm_data4, sfm_data5, sfm_data6, sfm_data7, sfm_data8,
		stlplus::create_filespec(sOutDir, "multicolorCameras", ".ply"),
		ESfM_Data(ALL));

	return EXIT_SUCCESS;
  

  std::cout << "success" << std::endl;
}
