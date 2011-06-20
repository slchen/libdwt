/* include PB2DFU library for BCE with FP01 DFU */
#include "../../api/00-pb-firmware/dfu_fp01_1x1.h"

void main()
{
	unsigned char len;

	pb2dfu_set_restart_addr(DFU_MEM_A, 0);
	pb2dfu_set_restart_addr(DFU_MEM_B, 0);
	pb2dfu_set_restart_addr(DFU_MEM_Z, 0);

	pb2dfu_set_inc(DFU_MEM_A, 2);
	pb2dfu_set_inc(DFU_MEM_Z, 2);

	while( len = pb2mb_read_data()>>1 )
	{
		// wal_bce_op(worker, WAL_PBID_P0, WAL_BCE_JK_VMULT,     0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, (len+1)/2 );
		pb2dfu_set_fulladdr(DFU_MEM_A, 0, 0);
		pb2dfu_set_fulladdr(DFU_MEM_B, 0, 0);
		pb2dfu_set_fulladdr(DFU_MEM_Z, 0, 0);
		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_start_op(DFU_OP_VMULT, len+1);
	
		// wal_bce_op(worker, WAL_PBID_P0, WAL_BCE_JK_VADD_AZ2B, 1, 0, 1, 0, 0, 0, 0, 0, 0, 2, 2, 2, (len-1)/2 );
		pb2dfu_set_fulladdr(DFU_MEM_A, 0, 1);
		pb2dfu_set_fulladdr(DFU_MEM_B, 0, 1);
		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_wait4hw();
		pb2dfu_start_op(DFU_OP_VADD_AZ2B, len);
	
		// wal_bce_op(worker, WAL_PBID_P0, WAL_BCE_JK_VADD_BZ2A, 1, 0, 1, 0, 2, 0, 0, 0, 0, 2, 2, 2, (len-1)/2 );
		pb2dfu_set_fulladdr(DFU_MEM_Z, 0, 2);
		pb2dfu_wait4hw();
		pb2dfu_start_op(DFU_OP_VADD_BZ2A, len);
		pb2dfu_wait4hw();
	
		pb2mb_eoc('.');
	}

	pb2mb_eoc('.');
	pb2mb_req_reset('.');
	pb2mb_reset();

	while (1)
		;
}
