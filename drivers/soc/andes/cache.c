// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/cacheinfo.h>
#include <linux/sizes.h>
#include <linux/smp.h>
#include <linux/irqflags.h>
#include <asm/csr.h>
#include <asm/sbi.h>
#include <asm/io.h>
#include <soc/andes/proc.h>
#include <soc/andes/csr.h>
#include <soc/andes/sbi.h>

#define MAX_CACHE_LINE_SIZE 256
#define EVSEL_MASK	0xff
#define SEL_PER_CTL	8
#define SEL_OFF(id)	(8 * (id % 8))

void __iomem *l2c_base;
/* default to offset of V0 memory map */
u32 L2C_REG_PER_CORE_OFFSET = 0x10;
u32 CCTL_L2_STATUS_PER_CORE_OFFSET = 0x4;
u32 L2C_REG_STATUS_OFFSET = 0;

DEFINE_PER_CPU(struct andesv5_cache_info, cpu_cache_info) = {
	.init_done = 0,
	.dcache_line_size = SZ_32
};

static void fill_cpu_cache_info(struct andesv5_cache_info *cpu_ci)
{
	int ncpu = get_cpu();
	struct cpu_cacheinfo *this_cpu_ci = get_cpu_cacheinfo(ncpu);
	struct cacheinfo *this_leaf = this_cpu_ci->info_list;
	unsigned int i = 0;

	for (; i < this_cpu_ci->num_leaves; i++, this_leaf++) {
		if (this_leaf->type == CACHE_TYPE_DATA) {
			cpu_ci->dcache_line_size =
				this_leaf->coherency_line_size;
		}
	}
	cpu_ci->init_done = true;
	put_cpu();
}

static inline int get_cache_line_size(void)
{
	int ncpu = get_cpu();
	struct andesv5_cache_info *cpu_ci = &per_cpu(cpu_cache_info, ncpu);

	if (unlikely(cpu_ci->init_done == false))
		fill_cpu_cache_info(cpu_ci);
	put_cpu();
	return cpu_ci->dcache_line_size;
}

static uint32_t cpu_l2c_get_cctl_status(void)
{
	int mhartid = get_cpu();

	put_cpu();
	return readl((void *)(l2c_base + L2C_REG_C0_STATUS_OFFSET +
		     mhartid * L2C_REG_STATUS_OFFSET));
}

void cpu_dcache_wb_range(unsigned long start, unsigned long end, int line_size,
			 struct page *page)
{
	int mhartid = get_cpu();
	unsigned long pa = page_to_phys(page);

	if (start & (~PAGE_MASK))
		pa += start & ~PAGE_MASK;

	while (end > start) {
		custom_csr_write(CCTL_REG_UCCTLBEGINADDR_NUM, start);
		custom_csr_write(CCTL_REG_UCCTLCOMMAND_NUM, CCTL_L1D_VA_WB);

		if (l2c_base) {
			writel(pa,
			       (void *)(l2c_base +
					L2C_REG_CN_ACC_OFFSET(mhartid)));

			writel(CCTL_L2_PA_WB,
			       (void *)(l2c_base +
					L2C_REG_CN_CMD_OFFSET(mhartid)));

			while ((cpu_l2c_get_cctl_status() &
				CCTL_L2_STATUS_CN_MASK(mhartid)) !=
				CCTL_L2_STATUS_IDLE)
				;
		}

		start += line_size;
		pa += line_size;
	}
	put_cpu();
}

void cpu_dcache_inval_range(unsigned long start, unsigned long end,
			    int line_size, struct page *page)
{
	int mhartid = get_cpu();
	unsigned long pa = page_to_phys(page);

	if (start & (~PAGE_MASK))
		pa += start & ~PAGE_MASK;

	while (end > start) {
		custom_csr_write(CCTL_REG_UCCTLBEGINADDR_NUM, start);
		custom_csr_write(CCTL_REG_UCCTLCOMMAND_NUM, CCTL_L1D_VA_INVAL);

		if (l2c_base) {
			writel(pa,
			       (void *)(l2c_base +
					L2C_REG_CN_ACC_OFFSET(mhartid)));

			writel(CCTL_L2_PA_INVAL,
			       (void *)(l2c_base +
					L2C_REG_CN_CMD_OFFSET(mhartid)));

			while ((cpu_l2c_get_cctl_status() &
				CCTL_L2_STATUS_CN_MASK(mhartid)) !=
				CCTL_L2_STATUS_IDLE)
				;
		}

		start += line_size;
		pa += line_size;
	}
	put_cpu();
}
void cpu_dma_inval_range(void *info)
{
	unsigned long flags;
	unsigned long line_size = get_cache_line_size();
	struct range_info *ri = info;
	unsigned long start = ri->start;
	unsigned long end = ri->end;
	unsigned long old_start = start;
	unsigned long old_end = end;
	char cache_buf[2][MAX_CACHE_LINE_SIZE] = {0};

	if (unlikely(start == end))
		return;

	start = start & (~(line_size - 1));
	end = ((end + line_size - 1) & (~(line_size - 1)));

	local_irq_save(flags);
	if (unlikely(start != old_start))
		memcpy(&cache_buf[0][0], (void *)start, line_size);

	if (unlikely(end != old_end)) {
		memcpy(&cache_buf[1][0], (void *)(old_end & (~(line_size - 1))),
		       line_size);
	}
	cpu_dcache_inval_range(start, end, line_size, ri->page);
	if (unlikely(start != old_start)) {
		memcpy((void *)start, &cache_buf[0][0],
		       (old_start & (line_size - 1)));
	}
	if (unlikely(end != old_end)) {
		memcpy((void *)(old_end + 1),
		       &cache_buf[1][(old_end & (line_size - 1)) + 1],
		       end - old_end - 1);
	}
	local_irq_restore(flags);
}
EXPORT_SYMBOL(cpu_dma_inval_range);

void cpu_dma_wb_range(void *info)
{
	unsigned long flags;
	unsigned long line_size = get_cache_line_size();
	struct range_info *ri = info;
	unsigned long start = ri->start;
	unsigned long end = ri->end;

	local_irq_save(flags);
	start = start & (~(line_size - 1));
	cpu_dcache_wb_range(start, end, line_size, ri->page);
	local_irq_restore(flags);
}
EXPORT_SYMBOL(cpu_dma_wb_range);

/* non-blocking load store */
long sbi_andes_get_non_blocking_status(void)
{
	struct sbiret ret;

	ret = sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_GET_MMISC_CTL_STATUS, 0, 0,
			0, 0, 0, 0);
	return ret.value;
}
void sbi_andes_set_mcache_ctl(unsigned long input)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_SET_MCACHE_CTL, input, 0, 0, 0,
		  0, 0);
	local_irq_restore(flags);
}

void sbi_andes_set_mmisc_ctl(unsigned long input)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_NON_BLOCKING_LOAD_STORE, input,
		  0, 0, 0, 0, 0);
	local_irq_restore(flags);
}

/* write around */
long sbi_andes_get_write_around_status(void)
{
	struct sbiret ret;

	ret = sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_GET_MCACHE_CTL_STATUS, 0,
			0, 0, 0, 0, 0);
	return ret.value;
}

void sbi_andes_enable_non_blocking_load_store(void)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_WRITE_AROUND, 1, 0, 0, 0, 0, 0);
	local_irq_restore(flags);
}

void sbi_andes_disable_non_blocking_load_store(void)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_WRITE_AROUND, 0, 0, 0, 0, 0, 0);
	local_irq_restore(flags);
}

void sbi_andes_enable_write_around(void)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_SET_MCACHE_CTL, 1, 0, 0,
		  0, 0, 0);
	local_irq_restore(flags);
}

void sbi_andes_disable_write_around(void)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_SET_MMISC_CTL, 0, 0, 0, 0, 0, 0);
	local_irq_restore(flags);
}

/* L1 Cache Prefetch */
void sbi_andes_enable_l1i_cache(void)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_L1CACHE_I_PREFETCH, 1, 0, 0, 0,
		  0, 0);
	local_irq_restore(flags);
}

void sbi_andes_disable_l1i_cache(void)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_L1CACHE_I_PREFETCH, 0, 0, 0, 0,
		  0, 0);
	local_irq_restore(flags);
}

void sbi_andes_enable_l1d_cache(void)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_L1CACHE_D_PREFETCH, 1, 0, 0, 0,
		  0, 0);
	local_irq_restore(flags);
}

void sbi_andes_disable_l1d_cache(void)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_L1CACHE_D_PREFETCH, 0, 0, 0, 0,
		  0, 0);
	local_irq_restore(flags);
}

/* L1 Cache */
long sbi_andes_cpu_l1c_status(void)
{
	struct sbiret ret;

	ret = sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_GET_MCACHE_CTL_STATUS, 0,
			0, 0, 0, 0, 0);
	return ret.value;
}

void sbi_andes_cpu_icache_enable(void *info)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_ICACHE_OP, 1, 0, 0, 0, 0, 0);
	local_irq_restore(flags);
}

void sbi_andes_cpu_icache_disable(void *info)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_ICACHE_OP, 0, 0, 0, 0, 0, 0);
	local_irq_restore(flags);
}

void sbi_andes_cpu_dcache_enable(void *info)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_DCACHE_OP, 1, 0, 0, 0, 0, 0);
	local_irq_restore(flags);
}

void sbi_andes_cpu_dcache_disable(void *info)
{
	unsigned long flags;

	local_irq_save(flags);
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_DCACHE_OP, 0, 0, 0, 0, 0, 0);
	local_irq_restore(flags);
}

int __init l2c_init(void)
{
	struct device_node *node;
	u32 l2c_cfg = 0;

	node = of_find_compatible_node(NULL, NULL, "cache");

	l2c_base = of_iomap(node, 0);
	if (l2c_base)
		l2c_cfg = *(u32 *)l2c_base;

	if (l2c_cfg & V5_L2C_CFG_MAP_MASK) {
		L2C_REG_PER_CORE_OFFSET = 0x1000;
		CCTL_L2_STATUS_PER_CORE_OFFSET = 0;
		L2C_REG_STATUS_OFFSET = 0x1000;
	}

	return 0;
}
arch_initcall(l2c_init)
