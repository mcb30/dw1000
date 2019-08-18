/*
 * inject.h
 *
 * Copyright (C) 2019 Petri Mattila <petri.mattila@unipart.io>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#ifndef __DW1000_INJECT_H__
#define __DW1000_INJECT_H__

#ifdef DW1000_ERROR_INJECT

enum {
	ERROR_INJECT_NONE = 0,
	ERROR_INJECT_STATE_DELAY,
	ERROR_INJECT_STATE_SYS_STATUS,
	ERROR_INJECT_TX_DELAY,
	ERROR_INJECT_TX_SPI_DATA,
	ERROR_INJECT_TX_SPI_INFO,
	ERROR_INJECT_RX_DELAY,
	ERROR_INJECT_RX_SYS_STATUS,
	ERROR_INJECT_SPI_RD_ERROR,
	ERROR_INJECT_SPI_WR_ERROR,
	ERROR_INJECT_CONFIG_ERROR,
	ERROR_INJECT_COUNT
};

#define INJECT_PRINT(_dw) \
        dev_dbg(_dw->dev, "Injecting %s:%d\n",                   \
                dw1000_inject_items[_dw->inject_error],          \
                _dw->inject_value)

#define INJECT_CLEAR(_dw) ({                                     \
        _dw->inject_error = 0;                                   \
        _dw->inject_value = 0;                                   \
})

#define INJECT_ERROR(_dw, _name, _args...) ({                    \
        if (_dw->inject_error == ERROR_INJECT_##_name) {         \
                INJECT_PRINT(_dw);                               \
                INJECT_ERROR_##_name(_dw, ##_args);              \
        }                                                        \
})

#define INJECT_ERROR_STATE_DELAY(_dw) ({                         \
        msleep(_dw->inject_value);                               \
        INJECT_CLEAR(_dw);                                       \
})

#define INJECT_ERROR_STATE_SYS_STATUS(_dw, _status) ({           \
        _status |= _dw->inject_value;                            \
        INJECT_CLEAR(_dw);                                       \
})

#define INJECT_ERROR_TX_DELAY(_dw) ({                            \
        msleep(_dw->inject_value);                               \
        INJECT_CLEAR(_dw);                                       \
})

#define INJECT_ERROR_RX_DELAY(_dw) ({                            \
        msleep(_dw->inject_value);                               \
        INJECT_CLEAR(_dw);                                       \
})

#define INJECT_ERROR_RX_SYS_STATUS(_dw, _status) ({              \
        _status = _dw->inject_value;                             \
        INJECT_CLEAR(_dw);                                       \
})

#define INJECT_ERROR_TX_SPI_DATA(_dw, _err) ({                   \
        dw1000_enqueue((_dw), DW1000_ERROR_WORK);                \
        INJECT_CLEAR(_dw);                                       \
        goto _err;                                               \
})

#define INJECT_ERROR_TX_SPI_INFO(_dw, _err) ({                   \
        dw1000_enqueue(_dw, DW1000_ERROR_WORK);                  \
        INJECT_CLEAR(_dw);                                       \
        goto _err;                                               \
})

#define INJECT_ERROR_SPI_RD_ERROR(_dw, _rc) ({                   \
        _rc = _dw->inject_value;                                 \
        INJECT_CLEAR(_dw);                                       \
})

#define INJECT_ERROR_SPI_WR_ERROR(_dw, _rc) ({                   \
        _rc = _dw->inject_value;                                 \
        INJECT_CLEAR(_dw);                                       \
})

#define INJECT_ERROR_CONFIG_ERROR(_dw, _rc, _err) ({             \
        _rc = _dw->inject_value;                                 \
        INJECT_CLEAR(_dw);                                       \
        goto _err;                                               \
})


#else /* DW1000_ERROR_INJECT */

#define INJECT_ERROR(_name, _args...)

#endif /* DW1000_ERROR_INJECT */

#endif /* __DW1000_INJECT_H__ */
