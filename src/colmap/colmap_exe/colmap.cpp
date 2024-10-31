#include <colmap/dll/colmap_lib.h>
#include <thread>
#include <iostream>
#include <filesystem>
#include <locale>
#include <corecrt_wstring.h>
#include <codecvt>

//#ifdef _WIN32
//#include <Windows.h>
//#endif
//std::string convert_str(const std::wstring& wide_str) {
//  //int len = MultiByteToWideChar(CP_UTF8, 0, utf8_str.c_str(), -1, nullptr, 0);
//  //std::wstring wide_str(len, 0);
//  //MultiByteToWideChar(CP_UTF8, 0, utf8_str.c_str(), -1, &wide_str[0], len);
//
//  auto len = WideCharToMultiByte(
//      CP_UTF8, 0, wide_str.c_str(), -1, nullptr, 0, nullptr, nullptr);
//  std::string mb_str(len, 0);
//  WideCharToMultiByte(
//      CP_UTF8, 0, wide_str.c_str(), -1, &mb_str[0], len, nullptr, nullptr);
//  if (mb_str[len - 1] == 0) {
//    mb_str = mb_str.substr(0, len - 1);
//  }
//  return mb_str;
//}
//
//inline std::string utf8_to_multibyte(const std::string& utf8_str) {
//#ifdef _WIN32
//  int len = MultiByteToWideChar(CP_UTF8, 0, utf8_str.c_str(), -1, nullptr, 0);
//  std::wstring wide_str(len, 0);
//  MultiByteToWideChar(CP_UTF8, 0, utf8_str.c_str(), -1, &wide_str[0], len);
//
//  len = WideCharToMultiByte(
//      CP_ACP, 0, wide_str.c_str(), -1, nullptr, 0, nullptr, nullptr);
//  std::string mb_str(len, 0);
//  WideCharToMultiByte(
//      CP_ACP, 0, wide_str.c_str(), -1, &mb_str[0], len, nullptr, nullptr);
//  if (mb_str[len - 1] == 0) {
//    mb_str = mb_str.substr(0, len - 1);
//  }
//  return mb_str;
//#else
//
//#endif
//}
int main() 
{ 
	
	ColmapSparseReconstruct sparse_reconstruct;
  sparse_reconstruct.option.image_path = "G:/tandt_db/3dgs_test_video/²ÝÊ¯/images";
  sparse_reconstruct.option.workspace_path ="G:/tandt_db/3dgs_test_video/²ÝÊ¯/";
  std::cout << "path: "
            << sparse_reconstruct.option.workspace_path << std::endl;
    sparse_reconstruct.option.quality = ColmapSparseReconstruct::Quality::High;
    sparse_reconstruct.option.use_glomap = false;
    sparse_reconstruct.option.output_sparse_points = true;
	std::thread t([&](){
    	sparse_reconstruct.run();
	});
	t.detach();

	while(1){
		//std::cout << "progress: " << sparse_reconstruct.GetProgressOnCurrentPhase() << std::flush;

		// std::cout << "";
                //const auto& p = sparse_reconstruct.getImageTracks(0);
		//std::cout << "size:" << p.size() << std::flush;
	}

	return 0;
}