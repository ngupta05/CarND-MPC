#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "matplotlibcpp.h"
// for convenience
namespace plt = matplotlibcpp;
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.rfind("}]");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
		int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

void plot(const vector<double>& x1, const vector<double>& y1, std::string t1,
		const vector<double>& x2, const vector<double>& y2, std::string t2,
		const vector<double>& x3, const vector<double>& y3, std::string t3
		) {
	plt::subplot(3, 1, 1);
	plt::title(t1.c_str());
	plt::plot(x1, y1);
	plt::subplot(3, 1, 2);
	plt::title(t2.c_str());
	plt::plot(x2, y2);
	plt::subplot(3, 1, 3);
	plt::title(t3.c_str());
	plt::plot(x3, y3);
	plt::show();
}

vector<double> world2car(double x0, double y0, double psi, double x, double y) {
	return {(x-x0)*cos(psi) + (y-y0)*sin(psi),
		(y-y0)*cos(psi) - (x-x0)*sin(psi)};
}

vector<double> map_wx, map_wy, map_t_wx, map_t_wy, map_mx, map_my;
std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

int main() {
	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;
	int counter = 0;

	h.onMessage([&mpc, &counter](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
			uWS::OpCode opCode) {
		counter++;
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		string sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			string s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry") {
					auto tp = std::chrono::system_clock::now();
					std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(tp - now).count()
							<< std::endl;
					now = tp;
					// j[1] is the data JSON object
					vector<double> ptsx = j[1]["ptsx"];
					vector<double> ptsy = j[1]["ptsy"];
					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					double v = j[1]["speed"]; // mph
					v = v * 1609.34 / 3600; // convert to m/s

					//map_wx.insert(map_wx.end(), ptsx.begin(), ptsx.end());
					//map_wy.insert(map_wy.end(), ptsy.begin(), ptsy.end());

					for (unsigned int i = 0; i < ptsx.size(); i++) {
						auto vars = world2car(px, py, psi, ptsx[i], ptsy[i]);
						ptsx[i] = vars[0];
						ptsy[i] = vars[1];
						//std::cout << vars[0] << "," << vars[1] << std::endl;
					}


					//map_t_wx.insert(map_t_wx.end(), ptsx.begin(), ptsx.end());
					//map_t_wy.insert(map_t_wy.end(), ptsy.begin(), ptsy.end());

					Eigen::VectorXd e_x_pts = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
					Eigen::VectorXd e_y_pts = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

					//std::cout << "Recv: " << j << std::endl;
					//int input; std::cin >> input;

					auto coeffs = polyfit(e_x_pts, e_y_pts, 2);

					double cte = polyeval(coeffs, 0) - 0;
					double epsi = 0 - atan(coeffs[1] + 2 * coeffs[2] * 0);

					Eigen::VectorXd state(6);
					state << 0, 0, 0, v, cte, epsi;

					auto vars = mpc.Solve(state, coeffs);

					if (vars.size() == 0) {
						return;
					}

					double steer_value = -1.0 * vars[6] / deg2rad(25);
					double throttle_value = vars[7];
					std::cout << "Actuations (d, a): " << steer_value << ","
							<< throttle_value << std::endl;

					json msgJson;
					// NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
					// Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle_value;

					//Display the MPC predicted trajectory
					vector<double> mpc_x_vals = mpc.get_mpc_x();
					vector<double> mpc_y_vals = mpc.get_mpc_y();

					//map_mx.insert(map_mx.end(), mpc_x_vals.begin(), mpc_x_vals.end());
					//map_my.insert(map_my.end(), mpc_y_vals.begin(), mpc_y_vals.end());

					//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
					// the points in the simulator are connected by a Green line

					msgJson["mpc_x"] = mpc_x_vals;
					msgJson["mpc_y"] = mpc_y_vals;


					//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
					// the points in the simulator are connected by a Yellow line

					msgJson["next_x"] = ptsx;
					msgJson["next_y"] = ptsy;


					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					//std::cout << msg << std::endl;
					// Latency
					// The purpose is to mimic real driving conditions where
					// the car does actuate the commands instantly.
					//
					// Feel free to play around with this value but should be to drive
					// around the track with 100ms latency.
					//
					// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
					// SUBMITTING.
					this_thread::sleep_for(chrono::milliseconds(100));
					if (counter > 0) {
						counter = 0;
						/*plot(map_wx, map_wy, "waypoints",
								map_t_wx, map_t_wy, "twaypoints",
								map_mx, map_my, "mpc");*/
					}
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
			size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
