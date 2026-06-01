#include <zephyr/sys/util.h>
#include "transport/transport.h"

#if IS_ENABLED(CONFIG_BT)
#include "transport/ble/rsc_service.h"
#endif

#if IS_ENABLED(CONFIG_ANT)
#include "transport/ant/ant_transport.h"
#endif

void transport_init(void)
{
#if IS_ENABLED(CONFIG_BT)
	rsc_service_init();
#endif
#if IS_ENABLED(CONFIG_ANT)
	ant_transport_init();
#endif
}

void transport_post(const sdm_data_t *data)
{
#if IS_ENABLED(CONFIG_BT)
	rsc_service_notify(data);
#endif
#if IS_ENABLED(CONFIG_ANT)
	ant_transport_post(data);
#endif
}
