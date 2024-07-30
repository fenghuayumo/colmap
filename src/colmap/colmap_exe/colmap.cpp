#include <colmap/dll/colmap_lib.h>

int main() 
{ 
	ColmapSparseReconstruct sparse_reconstruct;
	sparse_reconstruct.option.image_path = "G:/models/banana/images";
	sparse_reconstruct.option.workspace_path = "G:/models/banana";
    sparse_reconstruct.run();

	return 0;
}