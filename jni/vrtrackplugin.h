#ifdef WIN32
#define VRTRACKPLUGIN_API __declspec(dllexport) 

extern "C" {
	VRTRACKPLUGIN_API int startTrack();
	VRTRACKPLUGIN_API void stopTrack();
	VRTRACKPLUGIN_API void getPosition(float pos[]);
}
#endif