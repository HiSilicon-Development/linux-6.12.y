/* SPDX-License-Identifier: GPL-2.0-only */
/* Baseline JPEG syntax and JPGD table packing, derived from the HiSTB HAL. */
#ifndef HISTB_JPGD_JPEG_H
#define HISTB_JPGD_JPEG_H

struct histb_jpgd_huffman {
	u8 counts[16];
	u8 symbols[256];
	u16 count;
	bool present;
};

struct histb_jpgd_image {
	u32 width, height;
	u32 sampling, picture_type, mcu_size;
	u32 restart_interval, entropy_offset, stream_end;
	u32 quant[64], dc[12], ac_min[8], ac_base[8], ac_symbol[176];
};

struct histb_jpgd_parser {
	u8 quant[4][64];
	u8 quant_mask;
	struct histb_jpgd_huffman huff[2][4];
	u8 id[3], factor[3], qt[3], dc[3], ac[3], components;
	bool sof;
};

static u16 histb_jpgd_be16(const u8 *p)
{
	return p[0] << 8 | p[1];
}

static int histb_jpgd_dht(struct histb_jpgd_parser *p,
			const u8 *data, size_t len)
{
	while (len) {
		struct histb_jpgd_huffman *h;
		u32 code = 0, seen[8] = { 0 };
		unsigned int cls, id, n = 0, i, j;

		if (len < 17 || (data[0] & 0xec))
			return -EINVAL;
		cls = data[0] >> 4;
		id = data[0] & 3;
		h = &p->huff[cls][id];
		memset(h, 0, sizeof(*h));
		memcpy(h->counts, data + 1, 16);
		data += 17;
		len -= 17;
		for (i = 0; i < 16; i++) {
			n += h->counts[i];
			/* No oversubscribed tree or all-ones code (T.81 C.2). */
			if (code + h->counts[i] >= BIT(i + 1))
				return -EINVAL;
			code = (code + h->counts[i]) << 1;
		}
		if (!n || n > (cls ? 162 : 12) || n > len)
			return -EINVAL;
		for (j = 0; j < n; j++) {
			u8 s = data[j];

			if ((!cls && s > 11) ||
			    (cls && ((s & 15) > 10 ||
				      (!(s & 15) && s != 0 && s != 0xf0))) ||
			    (seen[s / 32] & BIT(s % 32)))
				return -EINVAL;
			seen[s / 32] |= BIT(s % 32);
		}
		memcpy(h->symbols, data, n);
		h->count = n;
		h->present = true;
		data += n;
		len -= n;
	}
	return 0;
}

static int histb_jpgd_pack_tables(struct histb_jpgd_parser *p,
				struct histb_jpgd_image *img)
{
	unsigned int bank, i, j, k;

	for (i = 0; i < p->components; i++) {
		if (!(p->quant_mask & BIT(p->qt[i])) ||
		    !p->huff[0][p->dc[i]].present ||
		    !p->huff[1][p->ac[i]].present)
			return -EINVAL;
	}
	/* The hardware has three quantizer lanes but only two Huffman lanes. */
	if (p->components == 3 &&
	    (memcmp(&p->huff[0][p->dc[1]], &p->huff[0][p->dc[2]],
		    sizeof(p->huff[0][0])) ||
	     memcmp(&p->huff[1][p->ac[1]], &p->huff[1][p->ac[2]],
		    sizeof(p->huff[1][0]))))
		return -EOPNOTSUPP;

	for (i = 0; i < 64; i++)
		for (j = 0; j < p->components; j++)
			img->quant[i] |= (u32)p->quant[p->qt[j]][i] << (j * 8);

	for (bank = 0; bank < (p->components == 1 ? 1 : 2); bank++) {
		struct histb_jpgd_huffman *dc = &p->huff[0][p->dc[bank]];
		struct histb_jpgd_huffman *ac = &p->huff[1][p->ac[bank]];
		u32 code = 0, ac_code = 0, n = 0;
		unsigned int last = 0;

		for (i = 0; i < 16; i++)
			if (ac->counts[i])
				last = i + 1;
		for (i = 0, k = 0; i < 16; i++) {
			u32 minimum = i < last ? ac_code : ~0U;
			u32 base = ac->counts[i] ? n - ac_code : 0;
			unsigned int shift = bank * 16 + (i & 1) * 8;

			img->ac_min[i / 2] |= (minimum & 0xff) << shift;
			img->ac_base[i / 2] |= (base & 0xff) << shift;
			for (j = 0; j < dc->counts[i]; j++, k++, code++)
				img->dc[dc->symbols[k]] |=
					(((i + 1) & 15) << 8 | (code & 0xff)) << (bank * 12);
			code <<= 1;
			n += ac->counts[i];
			ac_code = (ac_code + ac->counts[i]) << 1;
		}
		for (i = 0; i < ac->count; i++)
			img->ac_symbol[i] |= (u32)ac->symbols[i] << (bank * 8);
	}
	return 0;
}

static int histb_jpgd_parse(struct histb_jpgd_parser *p,
			  struct histb_jpgd_image *img, const u8 *data, size_t len)
{
	static const u8 zigzag[64] = {
		0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
		12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
		35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
		58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
	};
	size_t pos = 2;
	unsigned int i, j;

	memset(p, 0, sizeof(*p));
	memset(img, 0, sizeof(*img));
	if (len < 4 || data[0] != 0xff || data[1] != 0xd8)
		return -EINVAL;
	while (pos < len) {
		const u8 *s;
		size_t size;
		u8 marker;

		if (data[pos++] != 0xff)
			return -EINVAL;
		while (pos < len && data[pos] == 0xff)
			pos++;
		if (len - pos < 3)
			return -EINVAL;
		marker = data[pos++];
		size = histb_jpgd_be16(data + pos);
		if (size < 2 || size > len - pos)
			return -EINVAL;
		s = data + pos + 2;
		pos += size;
		size -= 2;
		switch (marker) {
		case 0xdb:
			if (!size)
				return -EINVAL;
			while (size) {
				u8 id = *s++;

				if (size < 65 || id > 3)
					return -EINVAL; /* baseline: 8-bit quantizers */
				for (i = 0; i < 64; i++) {
					if (!s[i])
						return -EINVAL;
					p->quant[id][zigzag[i]] = s[i];
				}
				p->quant_mask |= BIT(id);
				s += 64;
				size -= 65;
			}
			break;
		case 0xc4:
			if (!size || histb_jpgd_dht(p, s, size))
				return -EINVAL;
			break;
		case 0xc0:
			if (p->sof || size < 6 || s[0] != 8 ||
			    (s[5] != 1 && s[5] != 3) || size != 6 + 3 * s[5])
				return -EINVAL;
			p->sof = true;
			p->components = s[5];
			img->height = histb_jpgd_be16(s + 1);
			img->width = histb_jpgd_be16(s + 3);
			if (!img->width || !img->height ||
			    img->width > HISTB_JPGD_MAX_DIMENSION ||
			    img->height > HISTB_JPGD_MAX_DIMENSION)
				return -EINVAL;
			for (i = 0; i < p->components; i++) {
				p->id[i] = s[6 + i * 3];
				p->factor[i] = s[7 + i * 3];
				p->qt[i] = s[8 + i * 3];
				if (p->qt[i] > 3)
					return -EINVAL;
				for (j = 0; j < i; j++)
					if (p->id[j] == p->id[i])
						return -EINVAL;
			}
			if (p->components == 3 && p->id[0] == 'R' &&
			    p->id[1] == 'G' && p->id[2] == 'B')
				return -EOPNOTSUPP;
			if (p->components == 1) {
				if (p->factor[0] != 0x11)
					return -EOPNOTSUPP;
				img->picture_type = 0;
			} else {
				if (p->factor[1] != 0x11 || p->factor[2] != 0x11)
					return -EOPNOTSUPP;
				switch (p->factor[0]) {
				case 0x22: img->picture_type = 3; break;
				case 0x21: img->picture_type = 4; break;
				case 0x12: img->picture_type = 5; break;
				case 0x11: img->picture_type = 6; break;
				default: return -EOPNOTSUPP;
				}
			}
			img->sampling = p->factor[0] << 16 |
					p->factor[1] << 8 | p->factor[2];
			img->mcu_size = DIV_ROUND_UP(img->width, 8 * (p->factor[0] >> 4)) |
				DIV_ROUND_UP(img->height, 8 * (p->factor[0] & 15)) << 16;
			break;
		case 0xdd:
			if (size != 2)
				return -EINVAL;
			img->restart_interval = histb_jpgd_be16(s);
			break;
		case 0xda:
			if (!p->sof || size != 4 + 2 * p->components ||
			    s[0] != p->components)
				return -EOPNOTSUPP;
			for (i = 0; i < p->components; i++) {
				if (s[1 + 2 * i] != p->id[i] || (s[2 + 2 * i] & 0xcc))
					return -EOPNOTSUPP;
				p->dc[i] = s[2 + 2 * i] >> 4;
				p->ac[i] = s[2 + 2 * i] & 3;
			}
			if (s[size - 3] || s[size - 2] != 63 || s[size - 1])
				return -EOPNOTSUPP;
			img->entropy_offset = pos;
			/* Only a single complete scan, with stuffing and optional RSTs. */
			for (j = 0; pos < len;) {
				if (data[pos++] != 0xff)
					continue;
				while (pos < len && data[pos] == 0xff)
					pos++;
				if (pos == len)
					return -EINVAL;
				marker = data[pos++];
				if (!marker)
					continue;
				if (marker >= 0xd0 && marker <= 0xd7) {
					if (!img->restart_interval || marker != 0xd0 + (j++ & 7))
						return -EINVAL;
					continue;
				}
				if (marker != 0xd9 || pos == img->entropy_offset + 2)
					return -EINVAL;
				img->stream_end = pos;
				return histb_jpgd_pack_tables(p, img);
			}
			return -EINVAL;
		case 0xee:
			/* Adobe RGB/CMYK is not the YCbCr engine input contract. */
			if (size >= 12 && !memcmp(s, "Adobe", 5) && s[11] != 1)
				return -EOPNOTSUPP;
			break;
		default:
			if ((marker < 0xe0 || marker > 0xef) && marker != 0xfe)
				return -EOPNOTSUPP;
		}
	}
	return -EINVAL;
}
#endif
