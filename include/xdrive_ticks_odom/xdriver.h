#ifndef _H_XDRIVER
#define _H_XDRIVER

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

	#define TAG_NAME_VEL_LINE "VEL_LINE"
	#define TAG_NAME_VEL_THETA "VEL_THETA"

	#define MAKEIPADDRESS(b1,b2,b3,b4) ( ((unsigned int)b1<<24)+( (unsigned int)b2<<16)+ ((unsigned int)b3<<8)+b4 )

	#define CONTROLLER_IP MAKEIPADDRESS(192,168,1,33)

	float xdriver_getVel_line();
	bool xdriver_setVel_line(float vel);

	float xdriver_getVel_theta();
	bool xdriver_setVel_theta(float vel);

	unsigned int xdriver_getValue(const char *tagName);
	bool xdriver_setValue(const char *tagName, const unsigned int value);

	float xdriver_getValue_float(const char *tagName);
	bool xdriver_setValue_float(const char *tagName, const float value);

	unsigned int xdriver_struct_get(const char *tagName,void *structData);
	bool xdriver_struct_set(const char *tagName, void *structData);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
