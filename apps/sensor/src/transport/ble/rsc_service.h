#ifndef RSC_SERVICE_H_
#define RSC_SERVICE_H_

#include "sdm/sdm_data.h"

int rsc_service_init(void);
int rsc_service_notify(const sdm_data_t *data);

#endif /* RSC_SERVICE_H_ */
