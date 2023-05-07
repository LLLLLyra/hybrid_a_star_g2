#ifndef _TEST_H_
#define _TEST_H_
#include <fstream>
#include <iostream>

#include "../plan/hybrid_a_star.h"
#include "../proto/planer_config.pb.h"
#include "glog/logging.h"

void ResPrint(HybridAStartResult *res) {
  std::cout << "x = ";
  std::cout << '[';
  for (auto x : res->x) {
    std::cout << x << ", ";
  }
  std::cout << ']' << std::endl;

  std::cout << "y = ";
  std::cout << '[';
  for (auto x : res->y) {
    std::cout << x << ", ";
  }
  std::cout << ']' << std::endl;

  std::cout << "phi = ";
  std::cout << '[';
  for (auto x : res->phi) {
    std::cout << x << ", ";
  }
  std::cout << ']' << std::endl;

  std::cout << "v = ";
  std::cout << '[';
  for (auto x : res->v) {
    std::cout << x << ", ";
  }
  std::cout << ']' << std::endl;

  std::cout << "a = ";
  std::cout << '[';
  for (auto x : res->a) {
    std::cout << x << ", ";
  }
  std::cout << ']' << std::endl;
  std::cout << "steer = ";
  std::cout << '[';
  for (auto x : res->steer) {
    std::cout << x << ", ";
  }
  std::cout << ']' << std::endl;
};

// test0: initial demo
int test0() {
  google::InitGoogleLogging("test0");
  FLAGS_log_dir = "../log/";
  planing::PlannerOpenSpaceConfig const confg =
      planing::PlannerOpenSpaceConfig();
  HybridAStar h_a(confg);

  double sx = 117, sy = 52, sphi = 0.0, ex = 122, ey = 50, ephi = 0;
  std::vector<double> XYbounds = {60, 145, 48, 58};
  std::vector<std::vector<Vec2d>> obstacles;
  obstacles.push_back({Vec2d(98, 58), Vec2d(145, 58)});
  obstacles.push_back({Vec2d(98, 48), Vec2d(145, 48)});

  auto Rect = [&](double x, double y, double l, double w) -> void {
    obstacles.push_back({Vec2d(x, y), Vec2d(x + l, y)});
    obstacles.push_back({Vec2d(x, y), Vec2d(x, y - w)});
    obstacles.push_back({Vec2d(x + l, y), Vec2d(x + l, y - w)});
    obstacles.push_back({Vec2d(x, y - w), Vec2d(x + l, y - w)});
  };

  Rect(99, 58, 4.5, 2);
  Rect(105, 58, 4.5, 2);
  Rect(118, 58, 4.5, 2);
  Rect(124, 58, 4.5, 2);
  Rect(100, 50, 4.5, 2);
  Rect(110, 50, 4.5, 2);
  Rect(128, 50, 4.5, 2);

  HybridAStartResult res;

  bool sovled = h_a.Plan(sx, sy, sphi, ex, ey, ephi, XYbounds, obstacles, &res);
  if (sovled) {
    ResPrint(&res);
  } else {
    std::cout << "plan failed" << std::endl;
  }
  google::ShutdownGoogleLogging();
  return 0;
}

// test1
int test1() {
  google::InitGoogleLogging("test1");
  FLAGS_log_dir = "../log/";
  planing::PlannerOpenSpaceConfig const confg =
      planing::PlannerOpenSpaceConfig();
  HybridAStar h_a(confg);
  double sx = 2, sy = 2.25, sphi = 0.0, ex = 10, ey = -4, ephi = 1.57;
  std::vector<double> XYbounds = {-2, 22, -6, 10};
  std::vector<std::vector<Vec2d>> obstacles;
  obstacles.push_back({Vec2d(0, 8), Vec2d(12, 8)});
  obstacles.push_back({Vec2d(0, 0), Vec2d(8, 0)});
  obstacles.push_back({Vec2d(12, 0), Vec2d(20, 0)});
  HybridAStartResult res;
  bool sovled = h_a.Plan(sx, sy, sphi, ex, ey, ephi, XYbounds, obstacles, &res);
  if (sovled) {
    ResPrint(&res);
  } else {
    std::cout << "plan failed" << std::endl;
  }

  google::ShutdownGoogleLogging();
  return 0;
}

#endif