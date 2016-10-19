#include <linux/init.h>
#include <linux/module.h>

static int __init mbfs_init(void)
{
	pr_debug("mbfs module loaded\n");
	return 0;
}

static void __exit mbfs_fini(void)
{
	pr_debug("mbfs module unloaded\n");
}

module_init(mbfs_init);
module_exit(mbfs_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthias Beyer");

