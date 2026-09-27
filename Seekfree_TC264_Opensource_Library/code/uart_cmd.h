#ifndef _UART_CMD_H_
#define _UART_CMD_H_

#include "zf_common_headfile.h"

#ifndef UCMD_LINE_MAX
#define UCMD_LINE_MAX       64U
#endif

#ifndef UCMD_VAR_MAX
#define UCMD_VAR_MAX        16U
#endif

#ifndef UCMD_FIFO_BUFFER
#define UCMD_FIFO_BUFFER     128U
#endif

typedef enum
{
	UCMD_TYPE_INT32 = 0,
	UCMD_TYPE_INT16,
	UCMD_TYPE_FLOAT,
	UCMD_TYPE_DOUBLE,
}ucmd_type_t;

typedef struct
{
	const char *name;
	ucmd_type_t type;
	void *value;
	uint8 has_range;
	union
	{
		int32 min_int32;
		int16 min_int16;
		float min_float;
		double min_double;
	}min;
	union
	{
		int32 max_int32;
		int16 max_int16;
		float max_float;
		double max_double;
	}max;
}ucmd_var_t;

typedef struct
{
	ucmd_var_t vars[UCMD_VAR_MAX];
	uint8 var_count;
	char line[UCMD_LINE_MAX];
	uint32 line_length;
	uint8 line_discard;
	uint8 last_was_end;
	fifo_struct fifo;
	uint8 fifo_buf[UCMD_FIFO_BUFFER];
	uart_index_enum uart;
	void (*reply_fn)(const char *);
	uint8 initialized;
	volatile uint32 dropped_count;
}ucmd_ctx_t;

void  ucmd_init(ucmd_ctx_t *p, uart_index_enum uart, void (*reply_fn)(const char *));
uint8 ucmd_reg_int(ucmd_ctx_t *p, const char *name, int32 *value);
uint8 ucmd_reg_int_range(ucmd_ctx_t *p, const char *name, int32 *value, int32 min_value, int32 max_value);
uint8 ucmd_reg_int16(ucmd_ctx_t *p, const char *name, int16 *value);
uint8 ucmd_reg_int16_range(ucmd_ctx_t *p, const char *name, int16 *value, int16 min_value, int16 max_value);
uint8 ucmd_reg_float(ucmd_ctx_t *p, const char *name, float *value);
uint8 ucmd_reg_float_range(ucmd_ctx_t *p, const char *name, float *value, float min_value, float max_value);
uint8 ucmd_reg_double(ucmd_ctx_t *p, const char *name, double *value);
uint8 ucmd_reg_double_range(ucmd_ctx_t *p, const char *name, double *value, double min_value, double max_value);
void  ucmd_isr_feed_byte(ucmd_ctx_t *p, uint8 data);
void  ucmd_isr_feed(ucmd_ctx_t *p);
void  ucmd_poll(ucmd_ctx_t *p);

static inline void ucmd_send_text(ucmd_ctx_t *p, const char *text)
{
	if ((p != NULL) && (p->initialized != 0U) &&
		(p->reply_fn != NULL) && (text != NULL))
	{
		p->reply_fn(text);
	}
}

#endif
