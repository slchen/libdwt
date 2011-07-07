/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief PicoBlaze firmware performing 4 lifting steps and scaling after lifting together.
 */

// include PB2DFU library for BCE with FP01 DFU
#include "../../api/00-pb-firmware/dfu_fp01_1x1.h"

void main()
{
	unsigned char steps;

	pb2dfu_set_restart_addr(DFU_MEM_A, 0);
	pb2dfu_set_restart_addr(DFU_MEM_B, 0);
	pb2dfu_set_restart_addr(DFU_MEM_Z, 0);

	pb2dfu_set_bank(DFU_MEM_A, 0);
	pb2dfu_set_bank(DFU_MEM_B, 0);
	pb2dfu_set_bank(DFU_MEM_Z, 0);

	pb2dfu_set_inc(DFU_MEM_A, 2);
	pb2dfu_set_inc(DFU_MEM_Z, 2);

	while(steps = pb2mb_read_data())
	{
		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_A, 3);
		pb2dfu_set_addr(DFU_MEM_B, 6);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_set_cnt(steps); // HACK: should be "steps+1"
		pb2dfu_restart_op(DFU_OP_VMULT);

		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_A, 4);
		pb2dfu_set_addr(DFU_MEM_B, 1);
		pb2dfu_set_cnt(steps-1); // HACK: should be "steps"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_AZ2B);

		pb2dfu_set_addr(DFU_MEM_Z, 3);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_BZ2A);

		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_A, 2);
		pb2dfu_set_addr(DFU_MEM_B, 4);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_set_cnt(steps); // HACK: should be "steps+1"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VMULT);

		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_A, 3);
		pb2dfu_set_addr(DFU_MEM_B, 1);
		pb2dfu_set_cnt(steps-1); // HACK: should be "steps"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_AZ2B);

		pb2dfu_set_addr(DFU_MEM_Z, 3);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_BZ2A);

		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_A, 1);
		pb2dfu_set_addr(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_set_cnt(steps); // HACK: should be "steps+1"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VMULT);

		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_A, 2);
		pb2dfu_set_addr(DFU_MEM_B, 1);
		pb2dfu_set_cnt(steps-1); // HACK: should be "steps"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_AZ2B);

		pb2dfu_set_addr(DFU_MEM_Z, 3);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_BZ2A);

		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_A, 0);
		pb2dfu_set_addr(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_set_cnt(steps); // HACK: should be "steps+1"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VMULT);

		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_A, 1);
		pb2dfu_set_addr(DFU_MEM_B, 1);
		pb2dfu_set_cnt(steps-1); // HACK: should be "steps"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_AZ2B);

		pb2dfu_set_addr(DFU_MEM_Z, 3);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_BZ2A);

		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_set_addr(DFU_MEM_B, 8);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VMULT);

		pb2dfu_set_addr(DFU_MEM_B, 10);
		pb2dfu_set_addr(DFU_MEM_A, 0);
		pb2dfu_set_addr(DFU_MEM_Z, 0);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VMULT);

		pb2dfu_set_inc(DFU_MEM_A, 1);
		pb2dfu_set_inc(DFU_MEM_Z, 1);
		pb2dfu_set_cnt(2*steps-1); // HACK: should be "2*steps"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VZ2A);

		pb2dfu_set_inc(DFU_MEM_A, 2);
		pb2dfu_set_inc(DFU_MEM_Z, 2);
		pb2dfu_wait4hw();
		pb2mb_eoc('.');
	}

	pb2mb_eoc('.');
	pb2mb_req_reset('.');
	pb2mb_reset();

	while (1)
		;
}
