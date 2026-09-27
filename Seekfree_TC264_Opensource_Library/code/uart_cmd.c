#include "uart_cmd.h"

#include <errno.h>
#include <stdlib.h>

/* 查找已注册变量。 */
static ucmd_var_t *ucmd_find_var(ucmd_ctx_t *p, const char *name)
{
	uint8 index;

	for (index = 0U; index < p->var_count; index++)
	{
		if (strcmp(p->vars[index].name, name) == 0)
		{
			return &p->vars[index];
		}
	}
	return NULL;
}

/* 通过回复回调发送一条消息。 */
static void ucmd_reply(ucmd_ctx_t *p, const char *message)
{
	if ((p->reply_fn != NULL) && (message != NULL))
	{
		p->reply_fn(message);
	}
}

/* 校验并写入一个变量注册项。 */
static uint8 ucmd_add_var(ucmd_ctx_t *p, const char *name, ucmd_type_t type,
						  void *value, uint8 has_range, int32 min_int32,
						  int32 max_int32, double min_real, double max_real)
{
	ucmd_var_t *item;

	if ((p == NULL) || (name == NULL) || (value == NULL) ||
		(name[0] == '\0') || (p->var_count >= UCMD_VAR_MAX) ||
		(ucmd_find_var(p, name) != NULL))
	{
		return 1U;
	}

	item = &p->vars[p->var_count];
	item->name = name;
	item->type = type;
	item->value = value;
	item->has_range = has_range;
	if ((type == UCMD_TYPE_INT32) || (type == UCMD_TYPE_INT16))
	{
		item->min.min_int32 = min_int32;
		item->max.max_int32 = max_int32;
	}
	else if (type == UCMD_TYPE_FLOAT)
	{
		item->min.min_float = (float)min_real;
		item->max.max_float = (float)max_real;
	}
	else
	{
		item->min.min_double = min_real;
		item->max.max_double = max_real;
	}
	p->var_count++;
	return 0U;
}

/* 从命令行中取出下一个空白分隔的单词。 */
static char *ucmd_next_word(char **cursor)
{
	char *start;

	if ((cursor == NULL) || (*cursor == NULL))
	{
		return NULL;
	}

	while ((**cursor == ' ') || (**cursor == '\t'))
	{
		(*cursor)++;
	}
	if (**cursor == '\0')
	{
		return NULL;
	}

	start = *cursor;
	while ((**cursor != '\0') && (**cursor != ' ') && (**cursor != '\t'))
	{
		(*cursor)++;
	}
	if (**cursor != '\0')
	{
		**cursor = '\0';
		(*cursor)++;
	}
	return start;
}

/* 严格解析一个 int32 数值。 */
static uint8 ucmd_parse_int32(const char *text, int32 *value)
{
	char *end;
	long parsed;

	errno = 0;
	end = NULL;
	parsed = strtol(text, &end, 10);
	if ((errno == ERANGE) || (end == text) || (*end != '\0') ||
		(parsed < INT32_MIN) || (parsed > INT32_MAX))
	{
		return 1U;
	}
	*value = (int32)parsed;
	return 0U;
}

/* 严格解析一个有限 float 数值。 */
static uint8 ucmd_parse_float(const char *text, float *value)
{
	char *end;
	float parsed;

	errno = 0;
	end = NULL;
	parsed = strtof(text, &end);
	if ((errno == ERANGE) || (end == text) || (*end != '\0') || !isfinite(parsed))
	{
		return 1U;
	}
	*value = parsed;
	return 0U;
}

/* 严格解析一个有限 double 数值。 */
static uint8 ucmd_parse_double(const char *text, double *value)
{
	char *end;
	double parsed;

	errno = 0;
	end = NULL;
	parsed = strtod(text, &end);
	if ((errno == ERANGE) || (end == text) || (*end != '\0') || !isfinite(parsed))
	{
		return 1U;
	}
	*value = parsed;
	return 0U;
}

/* 将变量当前值格式化并回复给上位机。 */
static void ucmd_reply_value(ucmd_ctx_t *p, const char *name, const ucmd_var_t *item)
{
	char message[UCMD_LINE_MAX + 32U];

	if (item->type == UCMD_TYPE_INT32)
	{
		(void)snprintf(message, sizeof(message), "OK %s %ld\r\n", name,
					   (long)(*((int32 *)item->value)));
	}
	else if (item->type == UCMD_TYPE_INT16)
	{
		(void)snprintf(message, sizeof(message), "OK %s %d\r\n", name,
					   (int)(*((int16 *)item->value)));
	}
	else if (item->type == UCMD_TYPE_FLOAT)
	{
		(void)snprintf(message, sizeof(message), "OK %s %g\r\n", name,
					   (double)(*((float *)item->value)));
	}
	else
	{
		(void)snprintf(message, sizeof(message), "OK %s %g\r\n", name,
					   *((double *)item->value));
	}
	ucmd_reply(p, message);
}

/* 执行 name 后直接跟数值的紧凑命令，例如 spd10、lkp1.25。 */
static uint8 ucmd_process_compact(ucmd_ctx_t *p, const char *command)
{
	uint8 index;
	size_t name_length;
	size_t command_length;
	const char *value_text;
	ucmd_var_t *item;
	int32 int_value;
	float float_value;
	double double_value;
	uint32 interrupt_state;

	command_length = strlen(command);
	for (index = 0U; index < p->var_count; index++)
	{
		item = &p->vars[index];
		name_length = strlen(item->name);
		if ((command_length <= name_length) ||
			(strncmp(command, item->name, name_length) != 0))
		{
			continue;
		}

		value_text = &command[name_length];
		if ((item->type == UCMD_TYPE_INT32) || (item->type == UCMD_TYPE_INT16))
		{
			if (ucmd_parse_int32(value_text, &int_value) != 0U)
			{
				ucmd_reply(p, "ERR INVALID_VALUE\r\n");
				return 1U;
			}
			if ((item->type == UCMD_TYPE_INT16) &&
				((int_value < INT16_MIN) || (int_value > INT16_MAX)))
			{
				ucmd_reply(p, "ERR OUT_OF_RANGE\r\n");
				return 1U;
			}
			if ((item->has_range != 0U) &&
				((int_value < item->min.min_int32) || (int_value > item->max.max_int32)))
			{
				ucmd_reply(p, "ERR OUT_OF_RANGE\r\n");
				return 1U;
			}
			interrupt_state = interrupt_global_disable();
			if (item->type == UCMD_TYPE_INT32)
			{
				*((int32 *)item->value) = int_value;
			}
			else
			{
				*((int16 *)item->value) = (int16)int_value;
			}
			interrupt_global_enable(interrupt_state);
		}
		else if (item->type == UCMD_TYPE_FLOAT)
		{
			if (ucmd_parse_float(value_text, &float_value) != 0U)
			{
				ucmd_reply(p, "ERR INVALID_VALUE\r\n");
				return 1U;
			}
			if ((item->has_range != 0U) &&
				((float_value < item->min.min_float) || (float_value > item->max.max_float)))
			{
				ucmd_reply(p, "ERR OUT_OF_RANGE\r\n");
				return 1U;
			}
			interrupt_state = interrupt_global_disable();
			*((float *)item->value) = float_value;
			interrupt_global_enable(interrupt_state);
		}
		else
		{
			if (ucmd_parse_double(value_text, &double_value) != 0U)
			{
				ucmd_reply(p, "ERR INVALID_VALUE\r\n");
				return 1U;
			}
			if ((item->has_range != 0U) &&
				((double_value < item->min.min_double) || (double_value > item->max.max_double)))
			{
				ucmd_reply(p, "ERR OUT_OF_RANGE\r\n");
				return 1U;
			}
			interrupt_state = interrupt_global_disable();
			*((double *)item->value) = double_value;
			interrupt_global_enable(interrupt_state);
		}

		ucmd_reply_value(p, item->name, item);
		return 1U;
	}
	return 0U;
}

/* 解析并执行一整行命令。 */
static void ucmd_process_line(ucmd_ctx_t *p)
{
	char *cursor = p->line;
	char *command;
	char *name;
	char *value_text;
	char *extra;
	ucmd_var_t *item;
	int32 int_value;
	float float_value;

	if (p->line_discard != 0U)
	{
		ucmd_reply(p, "ERR LINE_TOO_LONG\r\n");
		return;
	}

	command = ucmd_next_word(&cursor);
	if (command == NULL)
	{
		return;
	}

	if (strcmp(command, "PING") == 0)
	{
		if (ucmd_next_word(&cursor) != NULL)
		{
			ucmd_reply(p, "ERR ARGUMENTS\r\n");
		}
		else
		{
			ucmd_reply(p, "PONG\r\n");
		}
		return;
	}

	if ((strcmp(command, "GET") != 0) && (strcmp(command, "SET") != 0))
	{
		if (ucmd_next_word(&cursor) != NULL)
		{
			ucmd_reply(p, "ERR ARGUMENTS\r\n");
		}
		else if (ucmd_process_compact(p, command) == 0U)
		{
			ucmd_reply(p, "ERR UNKNOWN_COMMAND\r\n");
		}
		return;
	}

	name = ucmd_next_word(&cursor);
	if (name == NULL)
	{
		ucmd_reply(p, "ERR ARGUMENTS\r\n");
		return;
	}
	item = ucmd_find_var(p, name);
	if (item == NULL)
	{
		ucmd_reply(p, "ERR UNKNOWN_VARIABLE\r\n");
		return;
	}

	if (strcmp(command, "GET") == 0)
	{
		if (ucmd_next_word(&cursor) != NULL)
		{
			ucmd_reply(p, "ERR ARGUMENTS\r\n");
		}
		else
		{
			ucmd_reply_value(p, name, item);
		}
		return;
	}

	value_text = ucmd_next_word(&cursor);
	extra = ucmd_next_word(&cursor);
	if ((value_text == NULL) || (extra != NULL))
	{
		ucmd_reply(p, "ERR ARGUMENTS\r\n");
		return;
	}

	if ((item->type == UCMD_TYPE_INT32) || (item->type == UCMD_TYPE_INT16))
	{
		if (ucmd_parse_int32(value_text, &int_value) != 0U)
		{
			ucmd_reply(p, "ERR INVALID_VALUE\r\n");
			return;
		}
		if ((item->has_range != 0U) &&
			((int_value < item->min.min_int32) || (int_value > item->max.max_int32)))
		{
			ucmd_reply(p, "ERR OUT_OF_RANGE\r\n");
			return;
		}
		if ((item->type == UCMD_TYPE_INT16) &&
			((int_value < INT16_MIN) || (int_value > INT16_MAX)))
		{
			ucmd_reply(p, "ERR OUT_OF_RANGE\r\n");
			return;
		}
		{
			uint32 interrupt_state = interrupt_global_disable();
			if (item->type == UCMD_TYPE_INT32)
			{
				*((int32 *)item->value) = int_value;
			}
			else
			{
				*((int16 *)item->value) = (int16)int_value;
			}
			interrupt_global_enable(interrupt_state);
		}
	}
	else if (item->type == UCMD_TYPE_FLOAT)
	{
		if (ucmd_parse_float(value_text, &float_value) != 0U)
		{
			ucmd_reply(p, "ERR INVALID_VALUE\r\n");
			return;
		}
		if ((item->has_range != 0U) &&
			((float_value < item->min.min_float) || (float_value > item->max.max_float)))
		{
			ucmd_reply(p, "ERR OUT_OF_RANGE\r\n");
			return;
		}
		{
			uint32 interrupt_state = interrupt_global_disable();
			*((float *)item->value) = float_value;
			interrupt_global_enable(interrupt_state);
		}
	}
	else
	{
		double double_value;
		if (ucmd_parse_double(value_text, &double_value) != 0U)
		{
			ucmd_reply(p, "ERR INVALID_VALUE\r\n");
			return;
		}
		if ((item->has_range != 0U) &&
			((double_value < item->min.min_double) || (double_value > item->max.max_double)))
		{
			ucmd_reply(p, "ERR OUT_OF_RANGE\r\n");
			return;
		}
		{
			uint32 interrupt_state = interrupt_global_disable();
			*((double *)item->value) = double_value;
			interrupt_global_enable(interrupt_state);
		}
	}
	ucmd_reply_value(p, name, item);
}

/* 初始化命令解析上下文和接收 FIFO。 */
void ucmd_init(ucmd_ctx_t *p, uart_index_enum uart, void (*reply_fn)(const char *))
{
	if (p == NULL)
	{
		return;
	}

	memset(p, 0, sizeof(*p));
	p->uart = uart;
	p->reply_fn = reply_fn;
	if (fifo_init(&p->fifo, FIFO_DATA_8BIT, p->fifo_buf, UCMD_FIFO_BUFFER) == FIFO_SUCCESS)
	{
		p->initialized = 1U;
	}
}

/* 注册一个无范围限制的 int32 变量。 */
uint8 ucmd_reg_int(ucmd_ctx_t *p, const char *name, int32 *value)
{
	return ucmd_add_var(p, name, UCMD_TYPE_INT32, value, 0U, 0, 0, 0.0f, 0.0f);
}

/* 注册一个带范围限制的 int32 变量。 */
uint8 ucmd_reg_int_range(ucmd_ctx_t *p, const char *name, int32 *value,
						 int32 min_value, int32 max_value)
{
	if (min_value > max_value)
	{
		return 1U;
	}
	return ucmd_add_var(p, name, UCMD_TYPE_INT32, value, 1U,
						min_value, max_value, 0.0f, 0.0f);
}

uint8 ucmd_reg_int16(ucmd_ctx_t *p, const char *name, int16 *value)
{
	return ucmd_add_var(p, name, UCMD_TYPE_INT16, value, 0U, 0, 0, 0.0, 0.0);
}

uint8 ucmd_reg_int16_range(ucmd_ctx_t *p, const char *name, int16 *value,
						   int16 min_value, int16 max_value)
{
	if (min_value > max_value)
	{
		return 1U;
	}
	return ucmd_add_var(p, name, UCMD_TYPE_INT16, value, 1U,
						min_value, max_value, 0.0, 0.0);
}

/* 注册一个无范围限制的 float 变量。 */
uint8 ucmd_reg_float(ucmd_ctx_t *p, const char *name, float *value)
{
	return ucmd_add_var(p, name, UCMD_TYPE_FLOAT, value, 0U, 0, 0, 0.0f, 0.0f);
}

/* 注册一个带范围限制的 float 变量。 */
uint8 ucmd_reg_float_range(ucmd_ctx_t *p, const char *name, float *value,
						   float min_value, float max_value)
{
	if (!isfinite(min_value) || !isfinite(max_value) || (min_value > max_value))
	{
		return 1U;
	}
	return ucmd_add_var(p, name, UCMD_TYPE_FLOAT, value, 1U, 0, 0,
						min_value, max_value);
}

uint8 ucmd_reg_double(ucmd_ctx_t *p, const char *name, double *value)
{
	return ucmd_add_var(p, name, UCMD_TYPE_DOUBLE, value, 0U, 0, 0, 0.0, 0.0);
}

uint8 ucmd_reg_double_range(ucmd_ctx_t *p, const char *name, double *value,
							double min_value, double max_value)
{
	if (!isfinite(min_value) || !isfinite(max_value) || (min_value > max_value))
	{
		return 1U;
	}
	return ucmd_add_var(p, name, UCMD_TYPE_DOUBLE, value, 1U, 0, 0,
						min_value, max_value);
}

/* 在接收中断中把已读取的一个字节写入 FIFO。 */
void ucmd_isr_feed_byte(ucmd_ctx_t *p, uint8 data)
{
	if ((p == NULL) || (p->initialized == 0U))
	{
		return;
	}
	if (fifo_write_element(&p->fifo, data) != FIFO_SUCCESS)
	{
		p->dropped_count++;
	}
}

/* 在独占串口接收中断中读取一个字节并写入 FIFO。 */
void ucmd_isr_feed(ucmd_ctx_t *p)
{
	if ((p == NULL) || (p->initialized == 0U))
	{
		return;
	}
	ucmd_isr_feed_byte(p, uart_read_byte(p->uart));
}

/* 在主循环中取出 FIFO 字节、组装并解析命令行。 */
void ucmd_poll(ucmd_ctx_t *p)
{
	uint8 data;
	fifo_state_enum read_state;
	uint32 interrupt_state;

	if ((p == NULL) || (p->initialized == 0U))
	{
		return;
	}

	while (1)
	{
		interrupt_state = interrupt_global_disable();
		if (fifo_used(&p->fifo) > 0U)
		{
			read_state = fifo_read_element(&p->fifo, &data,
				FIFO_READ_AND_CLEAN);
		}
		else
		{
			read_state = FIFO_DATA_NO_ENOUGH;
		}
		interrupt_global_enable(interrupt_state);

		if (read_state != FIFO_SUCCESS)
		{
			break;
		}

		if ((data == '\r') || (data == '\n'))
		{
			if (p->last_was_end == 0U)
			{
				p->line[p->line_length] = '\0';
				ucmd_process_line(p);
				p->line_length = 0U;
				p->line_discard = 0U;
			}
			p->last_was_end = 1U;
			continue;
		}

		p->last_was_end = 0U;
		if (p->line_discard == 0U)
		{
			if (p->line_length < (UCMD_LINE_MAX - 1U))
			{
				p->line[p->line_length++] = (char)data;
			}
			else
			{
				p->line_discard = 1U;
			}
		}
	}
}
