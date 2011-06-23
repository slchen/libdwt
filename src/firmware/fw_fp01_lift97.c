/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief PicoBlaze firmware performing two lifting steps together.
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

	while( steps = pb2mb_read_data() )
	{
		// V0:
		// A <= [Xoxoxoxo]
		// B <= [a-b-----]
		// steps <= 3

		// V1: Z[-O-O-O-O] <= A[-o-o-o-o] * B[a------.]
		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_A, 1);
		pb2dfu_set_addr(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_start_op(DFU_OP_VMULT, steps+1);

		// V2: B[a@b@-@--] <= A[--x-x-x-] + Z[-O-O-O-#]
		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_A, 2);
		pb2dfu_set_addr(DFU_MEM_B, 1);
		pb2dfu_wait4hw();
		pb2dfu_start_op(DFU_OP_VADD_AZ2B, steps);

		// V3: A[XoXoXoXo] <= B[a@b@-@--] + Z[-#-O-O-O]
		pb2dfu_set_addr(DFU_MEM_Z, 3);
		pb2dfu_wait4hw();
		pb2dfu_start_op(DFU_OP_VADD_BZ2A, steps);

		// V4: Z[Q-Q-Q-Q-] <= A[X-X-X-X-] * B[--b-----]
		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_Z, 0);
		pb2dfu_set_addr(DFU_MEM_A, 0);
		pb2dfu_wait4hw();
		pb2dfu_start_op(DFU_OP_VMULT, steps+1);

		// V5: B[a@b@-@--] <= A[-o-o-o--] + Z[Q-Q-Q-#-]
		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_A, 1);
		pb2dfu_set_addr(DFU_MEM_B, 1);
		pb2dfu_wait4hw();
		pb2dfu_start_op(DFU_OP_VADD_AZ2B, steps);

		// V6: A[XOXOXOXo] <= B[a@b@-@--] + Z[#-Q-Q-Q-]
		pb2dfu_set_addr(DFU_MEM_Z, 2);
		pb2dfu_wait4hw();
		pb2dfu_start_op(DFU_OP_VADD_BZ2A, steps);

		pb2dfu_wait4hw();

		pb2mb_eoc('.');
	}

	pb2mb_eoc('.');
	pb2mb_req_reset('.');
	pb2mb_reset();

	while (1)
		;
}
