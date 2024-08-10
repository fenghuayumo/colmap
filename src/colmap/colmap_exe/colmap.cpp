#include <colmap/dll/colmap_lib.h>

int main() 
{ 
	ColmapSparseReconstruct sparse_reconstruct;
	sparse_reconstruct.option.image_path = "G:/tandt_db/3dgs_test_video/sofa2/images";
	sparse_reconstruct.option.workspace_path = "G:/tandt_db/3dgs_test_video/sofa2/";
    sparse_reconstruct.option.use_glomap = true;
    sparse_reconstruct.option.output_sparse_points = true;
    sparse_reconstruct.run();

	return 0;
}