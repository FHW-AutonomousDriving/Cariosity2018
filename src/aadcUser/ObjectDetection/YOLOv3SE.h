﻿
/*
*  YOLOv3_SE.h
*  YOLOv3_SpringEdition
*
*  Created by kimbomm on 2018. 03. 31...
*  Copyright 2018 kimbomm. All rights reserved.
*
*/
#pragma GCC diagnostic ignored "-Wwrite-strings"

#include<fstream>
#include<string>
#include<vector>
#include<functional>
#include<exception>
#include<mutex>

/**
*	@brief 이 클래스는 cv::Rect를 확장한 것으로 클래스값과 스코어값이 추가되었습니다.
*	@author kimbomm
*	@date 2017-10-05
*
*/
class BoxSE : public cv::Rect {
public:
	int m_class = -1;
	float m_score = 0.0F;
	std::string m_class_name;
	BoxSE() {
		m_class_name = "Unknown";
	}
	BoxSE(int c, float s, int _x, int _y, int _w, int _h, std::string name = "")
		:m_class(c), m_score(s) {
		this->x = _x;
		this->y = _y;
		this->width = _w;
		this->height = _h;
		char* lb[5] = { "th","st","nd","rd","th" };
		if (name.length() == 0) {
			m_class_name = std::to_string(m_class) + lb[m_class < 4 ? m_class : 4] + " class";
		}
	}
};

class YOLOv3 {
private:
	typedef struct {
		int w;
		int h;
		int c;
		float *data;
	} image;
	using YoloLoadType = int*(*)(char* cfg, char* weights);
	using YoloTrainType = void(*)(char* _base_dir, char* _datafile, char* _cfgfile);
	using YoloDetectFromFileType = int(*)(char* img_path, int* _net, float threshold, float* result, int result_sz);
	using YoloDetectFromImageType = int(*)(float* data, int w, int h, int c, int* _net, float threshold, float* result, int result_sz);
	using YoloReleaseType = void(*)(int* net);
	using YoloClassifyFromFileType = int(*)(char* img_path, int* _net, float* result);
	using YoloClassifyFromImageType = int(*)(float* data, int w, int h, int c, int* _net, float* result);
private:
	YoloLoadType YoloLoad = nullptr;
	YoloTrainType YoloTrain = nullptr;
	YoloDetectFromFileType YoloDetectFromFile = nullptr;
	YoloDetectFromImageType YoloDetectFromImage = nullptr;
	YoloClassifyFromFileType YoloClassifyFromFile = nullptr;
	YoloClassifyFromImageType YoloClassifyFromImage = nullptr;
	YoloReleaseType YoloRelease=nullptr;
protected:
	int* m_network = nullptr;
#ifdef _WIN32
	HMODULE m_hmod = nullptr;
#else
	void* m_hmod = nullptr;
#endif
	std::vector<std::string> m_names;
public:
	std::string Names(size_t idx) {
		if (idx < m_names.size()) {
			return m_names[idx];
		} else {
			return "Unknown";
		}
	}
	void Create(std::string weights,std::string cfg,std::string names) {
		//Load
		//if (PathFileExistsA(weights.c_str()) == FALSE || PathFileExistsA(cfg.c_str()) == FALSE) {
		//	std::cerr << "Load Failed" << std::endl;
		//	exit(1);
		//}
		this->m_network = YoloLoad(const_cast<char*>(cfg.c_str()), const_cast<char*>(weights.c_str()));
		if (names.length() > 0) {
			std::fstream fin(names, std::ios::in);
			if (fin.is_open() == true) {
				this->m_names.clear();
				while (fin.eof() == false) {
					std::string str;
					std::getline(fin, str);
					if (str.length() > 0) {
						this->m_names.push_back(str);
					}
				}
				fin.close();
			}
		}
	}
	void Release() {
		if (this->m_hmod != nullptr) {
			YoloRelease(m_network);
#ifdef _WIN32
			FreeLibrary(this->m_hmod);
#else
			dlclose(this->m_hmod);
#endif
			m_hmod = nullptr;
		}
	}
	std::vector<BoxSE> Detect(cv::Mat img, float threshold) {
		IplImage* iplimg = new IplImage(img);
		std::vector<BoxSE> boxes= this->Detect(iplimg, threshold);
		delete iplimg;
		return boxes;
	}
	std::vector<BoxSE> Detect(std::string file, float threshold) {
		float result[6000] = { 0 };
		int n = YoloDetectFromFile(const_cast<char*>(file.c_str()), this->m_network, threshold, result, 6000);
		std::vector<BoxSE> boxes;
		for (int i = 0; i < n; i++) {
			BoxSE box;
			box.m_class = static_cast<int>(result[i * 6 + 0]);
			box.m_score = result[i * 6 + 1];
			box.x = static_cast<int>(result[i * 6 + 2]);
			box.y = static_cast<int>(result[i * 6 + 3]);
			box.width = static_cast<int>(result[i * 6 + 4]);
			box.height = static_cast<int>(result[i * 6 + 5]);
			if (this->m_names.size() > 0) {
				box.m_class_name = this->m_names[box.m_class];
			}
			boxes.push_back(box);
		}
		std::sort(boxes.begin(), boxes.end(), [](BoxSE a, BoxSE b)->bool { return a.m_score > b.m_score; });
		return boxes;
	}
	std::vector<BoxSE> Detect(IplImage* img, float threshold) {
		float result[6000] = { 0 };
		image im = ipl_to_image(img);
		int n = YoloDetectFromImage(im.data,im.w,im.h,im.c,this->m_network, threshold, result, 6000);
		free(im.data);
		std::vector<BoxSE> boxes;
		for (int i = 0; i < n; i++) {
			BoxSE box;
			box.m_class = static_cast<int>(result[i * 6 + 0]);
			box.m_score = result[i * 6 + 1];
			box.x = static_cast<int>(result[i * 6 + 2]);
			box.y = static_cast<int>(result[i * 6 + 3]);
			box.width = static_cast<int>(result[i * 6 + 4]);
			box.height = static_cast<int>(result[i * 6 + 5]);
			if (this->m_names.size() > 0) {
				box.m_class_name = this->m_names[box.m_class];
			}
			boxes.push_back(box);
		}
		std::sort(boxes.begin(), boxes.end(), [](BoxSE a, BoxSE b)->bool { return a.m_score > b.m_score; });
		return boxes;
	}

	int Classify(IplImage* img) {
		image im = ipl_to_image(img);
		int r=this->YoloClassifyFromImage(im.data, im.w, im.h, im.c, this->m_network, nullptr);
		free(im.data);
		return r;
	}
	int Classify(cv::Mat img) {
		IplImage* iplimg = new IplImage(img);
		int r = Classify(iplimg);
		delete iplimg;
		return r;
	}
	int Classify(std::string file) {
		return YoloClassifyFromFile(const_cast<char*>(file.c_str()), this->m_network, nullptr);
	}
	std::vector<BoxSE> GroundTruth(std::string image_file) {
		std::vector<BoxSE> ret;
		std::string txt_file = image_file.substr(0, image_file.find_last_of(".")) + ".txt";
		cv::Size size = cv::imread(image_file).size();
		std::fstream fin(txt_file, std::ios::in);
		while (fin.eof() == false) {
			std::string line;
			std::getline(fin, line);
			if (line.length() == 0)break;
			std::istringstream iss = std::istringstream(line);
			BoxSE box;
			iss >> box.m_class;
			float x1, y1, x2, y2;
			iss >> x1 >> y1 >> x2 >> y2;
			box.x = static_cast<int>((x1 - x2 / 2)*size.width);
			box.y = static_cast<int>((y1 - y2 / 2)*size.height);
			box.width = static_cast<int>(x2*size.width);
			box.height = static_cast<int>(y2*size.height);
			box.m_score = -1;
			ret.push_back(box);
		}
		fin.close();
		return ret;
	}
	YOLOv3() {
#ifdef _WIN32
		std::string dll = "libYOLOv3SE.dll";
		m_hmod = LoadLibraryA(dll.c_str());
		if (m_hmod == nullptr) {
			::MessageBoxA(NULL, "libYOLOv3SE.dll not found. or can't load dependency dll(cudnn64_7)", "Fatal", MB_OK);
			exit(1);
		}
		YoloLoad = (YoloLoadType)GetProcAddress(m_hmod, "YoloLoad");
		YoloTrain = (YoloTrainType)GetProcAddress(m_hmod, "YoloTrain");
		YoloDetectFromFile = (YoloDetectFromFileType)GetProcAddress(m_hmod, "YoloDetectFromFile");
		YoloDetectFromImage = (YoloDetectFromImageType)GetProcAddress(m_hmod, "YoloDetectFromImage");
		YoloClassifyFromFile = (YoloClassifyFromFileType)GetProcAddress(m_hmod, "YoloClassifyFromFile");
		YoloClassifyFromImage = (YoloClassifyFromImageType)GetProcAddress(m_hmod, "YoloClassifyFromImage");
		YoloRelease = (YoloReleaseType)GetProcAddress(m_hmod, "YoloRelease");
#else
		m_hmod = dlopen("libYOLOv3SE.so",RTLD_LAZY);
        if (m_hmod == nullptr) {
            std::cerr << dlerror() << std::endl;
            std::cerr << "libYOLOv3SE.so not found. or can't load dependency dlls" << std::endl;
            exit(1);
        }
        YoloLoad = (YoloLoadType)dlsym(m_hmod, "YoloLoad");
        YoloTrain = (YoloTrainType)dlsym(m_hmod, "YoloTrain");
        YoloDetectFromFile = (YoloDetectFromFileType)dlsym(m_hmod, "YoloDetectFromFile");
        YoloDetectFromImage = (YoloDetectFromImageType)dlsym(m_hmod, "YoloDetectFromImage");
        YoloRelease = (YoloReleaseType)dlsym(m_hmod, "YoloRelease");

#endif
    }
	~YOLOv3() {
		this->Release();
	}
private:
	image ipl_to_image(IplImage* src){
		image out;
		out.data = 0;
		out.h = src->height;
		out.w = src->width;
		out.c = src->nChannels;
		out.data = (float*)calloc(out.h*out.w*out.c, sizeof(float));
		unsigned char *data = (unsigned char *)src->imageData;
		int step = src->widthStep;
		int i, j, k;
		for (i = 0; i < out.h; ++i) {
			for (k = 0; k < out.c; ++k) {
				for (j = 0; j < out.w; ++j) {
					out.data[k*out.w*out.h + i*out.w + j] = data[i*step + j*out.c + k] / 255.F;
				}
			}
		}
		return out;
	}
};

