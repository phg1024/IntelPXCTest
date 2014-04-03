#include "Windows.h"
#include "util_render.h"
#include "util_pipeline.h"

#include "pxcprojection.h"
#include "pxcmetadata.h"

#include "FreeImagePlus.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <assert.h>
using namespace std;

template <typename T>
void dump2file(const string& filename, const vector<T>& data, int width, int height, int depth) {
	ofstream fout(filename, ios::binary);
	fout.write(reinterpret_cast<const char*>(&width), sizeof(int));
	fout.write(reinterpret_cast<const char*>(&height), sizeof(int));
	fout.write(reinterpret_cast<const char*>(&data[0]), sizeof(T)*width*height*depth);
	fout.close();
}

template <typename T>
vector<T> knn(const vector<T>& data, int width, int height, int ws) {
	vector<T> outdata(data.size());

	for(int i=0, idx=0;i<height;i++) {
		for(int j=0;j<width;j++, idx++) {
			if( data[idx] > 0 ) outdata[idx] = data[idx];
			else {
				// interpolation with nearest neighbors
				T vsum = 0;
				T wsum = 0;
				int hitcount = 0;
				const int minhit = 3;

				for(int nx=-ws;nx<=ws;nx++) {
					for(int ny=-ws;ny<=ws;ny++) {
						int x = j+nx;
						int y = i+ny;

						if( x<0 || x >= width ) continue;
						if( y<0 || y >= height ) continue;

						int vidx = y*width+x;
						T val = data[vidx];
						if( val > 0 ) {
							float w = sqrtf(nx*nx+ny*ny);
							vsum += val * w;
							wsum += w;
							hitcount++;
						}
					}
				}

				if( wsum > 0 && hitcount > minhit ) {
					outdata[idx] = vsum / wsum;
				}
				else outdata[idx] = -1;
			}
		}
	}

	return outdata;
}

int main(int argc, char* argv[]) {
	// use pipeline for aligned image
	UtilPipeline pflt;
    pflt.QueryCapture()->SetFilter(PXCCapture::Device::PROPERTY_DEPTH_SMOOTHING,true);		// use smoothed depth
	pflt.EnableImage(PXCImage::COLOR_FORMAT_RGB32);
    pflt.EnableImage(PXCImage::COLOR_FORMAT_DEPTH);
    pflt.Init();


	PXCSession* ss = pflt.QuerySession();
	PXCCapture::Device *dev = pflt.QueryCapture()->QueryDevice();

	UtilRender color_render(L"Color Stream");
	UtilRender depth_render(L"Depth Stream");

	for (int i=0;;i++) {
		if (!pflt.AcquireFrame(true)) break;
		PXCImage *color_image=pflt.QueryImage(PXCImage::IMAGE_TYPE_COLOR);
		PXCImage *depth_image=pflt.QueryImage(PXCImage::IMAGE_TYPE_DEPTH);
		
		// save the captured images
		if( i % 150 == 0 ) {
			stringstream ss;
			ss << (i / 150) << endl;
			string idxstr;
			ss >> idxstr;

			cout << "Capturing image #" << idxstr << endl;

			// use this to get raw buffer
			PXCImage::ImageData rgbData;
			PXCImage::ImageInfo rgbinfo;
			color_image->QueryInfo(&rgbinfo);
			assert( rgbinfo.format == PXCImage::COLOR_FORMAT_RGB24 );
			while( 1 ) {
				if( color_image->AcquireAccess(PXCImage::ACCESS_READ, &rgbData) == PXC_STATUS_NO_ERROR ) break;
			}


			// obtain the bgr image
#if 0
			vector<unsigned char> rgbimg(rgbinfo.width*rgbinfo.height*3);
			for(int i=0;i<rgbinfo.height;i++)
				memcpy((&rgbimg[0])+i*rgbinfo.width*3, rgbData.planes[0]+i*rgbData.pitches[0], rgbinfo.width*3*sizeof(unsigned char));
#else
			vector<unsigned char> rgbimg(rgbData.planes[0], rgbData.planes[0]+rgbinfo.width*rgbinfo.height*3);
#endif

			// save it to file
			dump2file<unsigned char>("color_" + idxstr + ".bin", rgbimg, rgbinfo.width, rgbinfo.height, 3);

			color_image->ReleaseAccess(&rgbData);

			PXCImage::ImageData depthData;
			PXCImage::ImageInfo depthinfo;
			depth_image->QueryInfo(&depthinfo);
			assert( depthinfo.format == PXCImage::COLOR_FORMAT_DEPTH );
			while( 1 ) {
				if( depth_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_DEPTH, &depthData) == PXC_STATUS_NO_ERROR ) break;
			}

			// obtain the depth image
			short* src  = (short*) depthData.planes[0];		

			float minDistance = 150;
			float maxDistance = 2000;
			float scaleZ = 1.0f;

#define MAP_TO_COLOR 1
#if MAP_TO_COLOR
			vector<float> depthimg(rgbinfo.width*rgbinfo.height, -1.0);

			// mapping from depth to color image, the mapped depth scatter around the color image space
			float *uvmap = (float*)depthData.planes[2];
			// process the image
			for (int i=0, idx=0; i<depthinfo.height; i++) {
				for (int j=0; j<depthinfo.width; j++, idx++) {
					float depth = (float)src[0];

					int x = uvmap[idx*2] * rgbinfo.width;
					int y = uvmap[idx*2+1] * rgbinfo.height;

					int cidx = y * rgbinfo.width + x;

					if ( depth < minDistance || depth > maxDistance ) { 
						// mark as bad point
						depthimg[cidx] = -1.0;
					}
					else {

						// normalize depth, not necessary
						//depth = (depth-minDistance) / (maxDistance-minDistance);
						if( depthimg[cidx] < 0 ) depthimg[cidx] = depth;
						else depthimg[cidx] = min(depth, depthimg[cidx]);
					}
					src ++;
				}
			}

			// apply knn filters to the sparse depth map, with decreasing window size
			depthimg = knn(depthimg, rgbinfo.width, rgbinfo.height, 3);
			depthimg = knn(depthimg, rgbinfo.width, rgbinfo.height, 2);
			depthimg = knn(depthimg, rgbinfo.width, rgbinfo.height, 1);
			// save it to file
			dump2file<float>("depth_" + idxstr + ".bin", depthimg, rgbinfo.width, rgbinfo.height, 1);
#else
			vector<float> depthimg(depthinfo.width*depthinfo.height, -1);

			// mapping from depth to color image, the mapped depth scatter around the color image space
			float *uvmap = (float*)depthData.planes[2];
			// process the image
			for (int i=0, idx=0; i<depthinfo.height; i++) {
				for (int j=0; j<depthinfo.width; j++, idx++) {
					float depth = (float)src[0];

					if ( depth < minDistance || depth > maxDistance ) { 
						// mark as bad point
						depth = -1.0;
					}

					// normalize depth, not necessary
					//depth = (depth-minDistance) / (maxDistance-minDistance);
					depthimg[idx] = depth;
					src ++;
				}
			}
			// save it to file
			dump2file<float>("depth_" + idxstr + ".bin", depthimg, depthinfo.width, depthinfo.height, 1);
#endif

			depth_image->ReleaseAccess(&depthData);
		}

		if (!color_render.RenderFrame(color_image)) break;
		if (!depth_render.RenderFrame(depth_image)) break;

		pflt.ReleaseFrame();
	}
	pflt.Close();
	return 0;
}