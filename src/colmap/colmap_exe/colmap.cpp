#include <colmap/dll/colmap_lib.h>
#include <thread>
#include <iostream>
int main() 
{ 
	ColmapSparseReconstruct sparse_reconstruct;
	sparse_reconstruct.option.image_path = "G:/tandt_db/3dgs_test_video/sofa2/images";
	sparse_reconstruct.option.workspace_path = "G:/tandt_db/3dgs_test_video/sofa2/";
    sparse_reconstruct.option.use_glomap = true;
    sparse_reconstruct.option.output_sparse_points = true;
	std::thread t([&](){
    	sparse_reconstruct.run();
	});
	t.detach();

	while(1){
		//std::cout << "progress: " << sparse_reconstruct.GetProgressOnCurrentPhase() << std::flush;

		// std::cout << "";
                const auto& p = sparse_reconstruct.getImageTracks(0);
		std::cout << "size:" << p.size() << std::flush;
	}

	return 0;
}