/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief PicoBlaze firmware performing 4 lifting steps and scaling after lifting together.
 */

#include "../../api/20-pb-firmware/pbbcelib.h"

// TODO: elliminated DFU_VCOPY operation

int main()
{
	pb2mb_report_running();

	unsigned int steps;
	unsigned int pb_offset;

	/*
	pb2dfu_set_restart_addr(DFU_MEM_A, 0);
	pb2dfu_set_restart_addr(DFU_MEM_B, 0);
	pb2dfu_set_restart_addr(DFU_MEM_Z, 0);

	pb2dfu_set_bank(DFU_MEM_A, 0);
	pb2dfu_set_bank(DFU_MEM_B, 0);
	pb2dfu_set_bank(DFU_MEM_Z, 0);

	pb2dfu_set_inc(DFU_MEM_A, 2);
	pb2dfu_set_inc(DFU_MEM_Z, 2);

	while(steps = pb2mb_read_data())
	*/
	while( mbpb_exchange_data(0) )
	{
		steps = read_bce_cmem_u16(0x01, 0);
		pb_offset = read_bce_cmem_u16(0x02, 0);

		/*
		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_A, 3);
		pb2dfu_set_addr(DFU_MEM_B, 6);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_set_cnt(steps); // HACK: should be "steps+1"
		pb2dfu_restart_op(DFU_OP_VMULT);
		*/
		// TODO: Z[1:2:] <= A[3:2:] * B[6:0:] (cnt=steps+1)
		// TODO: Z[1:2:] <= A[pb_offset+3:2:] * B[6:0:] (cnt=steps+1)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_C); // C = Z
		pb2dfu_set_bank(DFUAG_1, MBANK_A);
		pb2dfu_set_bank(DFUAG_2, MBANK_B);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*Z*/, 1);
		pb2dfu_set_addr(DFUAG_1/*A*/, pb_offset+3);
		pb2dfu_set_addr(DFUAG_2/*B*/, 6);
		// incrementy
		pb2dfu_set_inc(DFUAG_0/*Z*/, 2);
		pb2dfu_set_inc(DFUAG_1/*A*/, 2);
		pb2dfu_set_inc(DFUAG_2/*B*/, 0);
		// cnt
		pb2dfu_set_cnt(steps+1);
		// operation
		pb2dfu_restart_op(DFU_VMUL);

		/*
		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_A, 4);
		pb2dfu_set_addr(DFU_MEM_B, 1);
		pb2dfu_set_cnt(steps-1); // HACK: should be "steps"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_AZ2B);
		*/
		// TODO: B[1:2:] <= A[4:2:] + Z[1:2:] (cnt=steps)
		// TODO: B[1:2:] <= A[pb_offset+4:2:] + Z[1:2:] (cnt=steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_B);
		pb2dfu_set_bank(DFUAG_1, MBANK_A);
		pb2dfu_set_bank(DFUAG_2, MBANK_C); // C = Z
		// offsety
		pb2dfu_set_addr(DFUAG_0/*B*/, 1);
		pb2dfu_set_addr(DFUAG_1/*A*/, pb_offset+4);
		pb2dfu_set_addr(DFUAG_2/*Z*/, 1);
		// incrementy
		pb2dfu_set_inc(DFUAG_0/*B*/, 2);
		pb2dfu_set_inc(DFUAG_1/*A*/, 2);
		pb2dfu_set_inc(DFUAG_2/*Z*/, 2);
		// cnt
		pb2dfu_set_cnt(steps);
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VADD);

		/*
		pb2dfu_set_addr(DFU_MEM_Z, 3);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_BZ2A);
		*/
		// TODO: A[4:2:] <= B[1:2:] + Z[3:2:] (cnt=steps)
		// TODO: A[pb_offset+4:2:] <= B[1:2:] + Z[3:2:] (cnt=steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_A);
		pb2dfu_set_bank(DFUAG_1, MBANK_B);
		pb2dfu_set_bank(DFUAG_2, MBANK_C); // C = Z
		// offsety
		pb2dfu_set_addr(DFUAG_0/*A*/, pb_offset+4);
		pb2dfu_set_addr(DFUAG_1/*B*/, 1);
		pb2dfu_set_addr(DFUAG_2/*Z*/, 3);
		// incrementy
		pb2dfu_set_inc(DFUAG_0/*A*/, 2);
		pb2dfu_set_inc(DFUAG_1/*B*/, 2);
		pb2dfu_set_inc(DFUAG_2/*Z*/, 2);
		// cnt
		pb2dfu_set_cnt(steps);
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VADD);

		/*
		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_A, 2);
		pb2dfu_set_addr(DFU_MEM_B, 4);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_set_cnt(steps); // HACK: should be "steps+1"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VMULT);
		*/
		// TODO: Z[1:2:] <= A[2:2:] * B[4:0:] (cnt=steps+1)
		// TODO: Z[1:2:] <= A[pb_offset+2:2:] * B[4:0:] (cnt=steps+1)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_C); // C = Z
		pb2dfu_set_bank(DFUAG_1, MBANK_A);
		pb2dfu_set_bank(DFUAG_2, MBANK_B);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*Z*/, 1);
		pb2dfu_set_addr(DFUAG_1/*A*/, pb_offset+2);
		pb2dfu_set_addr(DFUAG_2/*B*/, 4);
		// incrementy
		pb2dfu_set_inc(DFUAG_0/*Z*/, 2);
		pb2dfu_set_inc(DFUAG_1/*A*/, 2);
		pb2dfu_set_inc(DFUAG_2/*B*/, 0);
		// cnt
		pb2dfu_set_cnt(steps+1);
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VMUL);

		/*
		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_A, 3);
		pb2dfu_set_addr(DFU_MEM_B, 1);
		pb2dfu_set_cnt(steps-1); // HACK: should be "steps"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_AZ2B);
		*/
		// TODO: B[1:2:] <= A[3:2:] + Z[1:2:] (cnt=steps)
		// TODO: B[1:2:] <= A[pb_offset+3:2:] + Z[1:2:] (cnt=steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_B);
		pb2dfu_set_bank(DFUAG_1, MBANK_A);
		pb2dfu_set_bank(DFUAG_2, MBANK_C);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*B*/, 1);
		pb2dfu_set_addr(DFUAG_1/*A*/, pb_offset+3);
		pb2dfu_set_addr(DFUAG_2/*Z*/, 1);
		// incrementy
		pb2dfu_set_inc(DFUAG_0/*B*/, 2);
		pb2dfu_set_inc(DFUAG_1/*A*/, 2);
		pb2dfu_set_inc(DFUAG_2/*Z*/, 2);
		// cnt
		pb2dfu_set_cnt(steps); // FIXME: spadne pblaze-llc: Assertion `DestReg == VirtReg && "Unknown load situation!"' failed.
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VADD);

		/*
		pb2dfu_set_addr(DFU_MEM_Z, 3);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_BZ2A);
		*/
		// TODO: A[3:2:] <= B[1:2:] + Z[3:2:] (cnt=steps)
		// TODO: A[pb_offset+3:2:] <= B[1:2:] + Z[3:2:] (cnt=steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_A);
		pb2dfu_set_bank(DFUAG_1, MBANK_B);
		pb2dfu_set_bank(DFUAG_2, MBANK_C);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*A*/, pb_offset+3);
		pb2dfu_set_addr(DFUAG_1/*B*/, 1);
		pb2dfu_set_addr(DFUAG_2/*Z*/, 3);
		// incerementy
		pb2dfu_set_inc(DFUAG_0/*A*/, 2);
		pb2dfu_set_inc(DFUAG_1/*B*/, 2);
		pb2dfu_set_inc(DFUAG_2/*Z*/, 2);
		// cnt
		pb2dfu_set_cnt(steps); // FIXME: spadne pblaze-llc: Assertion `DestReg == VirtReg && "Unknown load situation!"' failed.
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VADD);

		/*
		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_A, 1);
		pb2dfu_set_addr(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_set_cnt(steps); // HACK: should be "steps+1"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VMULT);
		*/
		// TODO: Z[1:2:] <= A[1:2:] * B[2:0:] (cnt=steps+1)
		// TODO: Z[1:2:] <= A[pb_offset+1:2:] * B[2:0:] (cnt=steps+1)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_C);
		pb2dfu_set_bank(DFUAG_1, MBANK_A);
		pb2dfu_set_bank(DFUAG_2, MBANK_B);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*Z*/, 1);
		pb2dfu_set_addr(DFUAG_1/*A*/, pb_offset+1); // FIXME: Assertion `DestReg == VirtReg && "Unknown load situation!"' failed.
		pb2dfu_set_addr(DFUAG_2/*B*/, 2);
		// incerementy
		pb2dfu_set_inc(DFUAG_0/*Z*/, 2);
		pb2dfu_set_inc(DFUAG_1/*A*/, 2);
		pb2dfu_set_inc(DFUAG_2/*B*/, 0);
		// cnt
		pb2dfu_set_cnt(steps+1);
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VMUL);

		/*
		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_A, 2);
		pb2dfu_set_addr(DFU_MEM_B, 1);
		pb2dfu_set_cnt(steps-1); // HACK: should be "steps"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_AZ2B);
		*/
		// TODO: B[1:2:] <= A[2:2:] + Z[1:2:] (cnt=steps)
		// TODO: B[1:2:] <= A[pb_offset+2:2:] + Z[1:2:] (cnt=steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_B);
		pb2dfu_set_bank(DFUAG_1, MBANK_A);
		pb2dfu_set_bank(DFUAG_2, MBANK_C);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*B*/, 1);
		pb2dfu_set_addr(DFUAG_1/*A*/, pb_offset+2);
		pb2dfu_set_addr(DFUAG_2/*Z*/, 1);
		// incerementy
		pb2dfu_set_inc(DFUAG_0/*B*/, 2);
		pb2dfu_set_inc(DFUAG_1/*A*/, 2);
		pb2dfu_set_inc(DFUAG_2/*Z*/, 2);
		// cnt
		pb2dfu_set_cnt(steps); // FIXME: spadne pblaze-llc: Assertion `DestReg == VirtReg && "Unknown load situation!"' failed.
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VADD);

		/*
		pb2dfu_set_addr(DFU_MEM_Z, 3);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_BZ2A);
		*/
		// TODO: A[2:2:] <= B[1:2:] + Z[3:2:] (cnt=steps)
		// TODO: A[pb_offset+2:2:] <= B[1:2:] + Z[3:2:] (cnt=steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_A);
		pb2dfu_set_bank(DFUAG_1, MBANK_B);
		pb2dfu_set_bank(DFUAG_2, MBANK_C);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*A*/, pb_offset+2);
		pb2dfu_set_addr(DFUAG_1/*B*/, 1);
		pb2dfu_set_addr(DFUAG_2/*Z*/, 3);
		// incerementy
		pb2dfu_set_inc(DFUAG_0/*A*/, 2);
		pb2dfu_set_inc(DFUAG_1/*B*/, 2);
		pb2dfu_set_inc(DFUAG_2/*Z*/, 2);
		// cnt
		pb2dfu_set_cnt(steps); // FIXME: spadne pblaze-llc: Assertion `DestReg == VirtReg && "Unknown load situation!"' failed.
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VADD);

		/*
		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_A, 0);
		pb2dfu_set_addr(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_set_cnt(steps); // HACK: should be "steps+1"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VMULT);
		*/
		// TODO: Z[1:2:] <= A[0:2:] * B[0:0:] (cnt=steps+1)
		// TODO: Z[1:2:] <= A[pb_offset+0:2:] * B[0:0:] (cnt=steps+1)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_C);
		pb2dfu_set_bank(DFUAG_1, MBANK_A);
		pb2dfu_set_bank(DFUAG_2, MBANK_B);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*Z*/, 1);
		pb2dfu_set_addr(DFUAG_1/*A*/, pb_offset+0);
		pb2dfu_set_addr(DFUAG_2/*B*/, 0);
		// incerementy
		pb2dfu_set_inc(DFUAG_0/*Z*/, 2);
		pb2dfu_set_inc(DFUAG_1/*A*/, 2);
		pb2dfu_set_inc(DFUAG_2/*B*/, 0);
		// cnt
		pb2dfu_set_cnt(steps+1);
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VMUL);

		/*
		pb2dfu_set_inc(DFU_MEM_B, 2);
		pb2dfu_set_addr(DFU_MEM_A, 1);
		pb2dfu_set_addr(DFU_MEM_B, 1);
		pb2dfu_set_cnt(steps-1); // HACK: should be "steps"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_AZ2B);
		*/
		// TODO: B[1:2:] <= A[1:2:] + Z[1:2:] (cnt=steps)
		// TODO: B[1:2:] <= A[pb_offset+1:2:] + Z[1:2:] (cnt=steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_B);
		pb2dfu_set_bank(DFUAG_1, MBANK_A);
		pb2dfu_set_bank(DFUAG_2, MBANK_C);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*B*/, 1);
		pb2dfu_set_addr(DFUAG_1/*A*/, pb_offset+1);
		pb2dfu_set_addr(DFUAG_2/*Z*/, 1);
		// incerementy
		pb2dfu_set_inc(DFUAG_0/*B*/, 2);
		pb2dfu_set_inc(DFUAG_1/*A*/, 2);
		pb2dfu_set_inc(DFUAG_2/*Z*/, 2);
		// cnt
		pb2dfu_set_cnt(steps); // FIXME: spadne pblaze-llc: Assertion `DestReg == VirtReg && "Unknown load situation!"' failed.
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VADD);

		/*
		pb2dfu_set_addr(DFU_MEM_Z, 3);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VADD_BZ2A);
		*/
		// TODO: A[1:2:] <= B[1:2:] + Z[3:2:] (cnt=steps)
		// TODO: A[pb_offset+1:2:] <= B[1:2:] + Z[3:2:] (cnt=steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_A);
		pb2dfu_set_bank(DFUAG_1, MBANK_B);
		pb2dfu_set_bank(DFUAG_2, MBANK_C);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*A*/, pb_offset+1);
		pb2dfu_set_addr(DFUAG_1/*B*/, 1);
		pb2dfu_set_addr(DFUAG_2/*Z*/, 3);
		// incerementy
		pb2dfu_set_inc(DFUAG_0/*A*/, 2);
		pb2dfu_set_inc(DFUAG_1/*B*/, 2);
		pb2dfu_set_inc(DFUAG_2/*Z*/, 2);
		// cnt
		pb2dfu_set_cnt(steps); // FIXME: spadne pblaze-llc: Assertion `DestReg == VirtReg && "Unknown load situation!"' failed.
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VADD);

		/*
		pb2dfu_set_inc(DFU_MEM_B, 0);
		pb2dfu_set_addr(DFU_MEM_Z, 1);
		pb2dfu_set_addr(DFU_MEM_B, 8);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VMULT);
		*/
		// TODO: Z[1:2:] <= A[1:2:] * B[8:0:] (cnt=steps)
		// TODO: Z[1:2:] <= A[pb_offset+1:2:] * B[8:0:] (cnt=steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_C);
		pb2dfu_set_bank(DFUAG_1, MBANK_A);
		pb2dfu_set_bank(DFUAG_2, MBANK_B);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*Z*/, 1);
		pb2dfu_set_addr(DFUAG_1/*A*/, pb_offset+1);
		pb2dfu_set_addr(DFUAG_2/*B*/, 8);
		// incrementy
		pb2dfu_set_inc(DFUAG_0/*Z*/, 2);
		pb2dfu_set_inc(DFUAG_1/*A*/, 2);
		pb2dfu_set_inc(DFUAG_2/*B*/, 0);
		// cnt
		pb2dfu_set_cnt(steps); // FIXME: spadne pblaze-llc: Assertion `DestReg == VirtReg && "Unknown load situation!"' failed.
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VMUL);

		/*
		pb2dfu_set_addr(DFU_MEM_B, 10);
		pb2dfu_set_addr(DFU_MEM_A, 0);
		pb2dfu_set_addr(DFU_MEM_Z, 0);
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VMULT);
		*/
		// TODO: Z[0:2:] <= A[0:2:] * B[10:0:] (cnt=steps)
		// TODO: Z[0:2:] <= A[pb_offset+0:2:] * B[10:0:] (cnt=steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_C);
		pb2dfu_set_bank(DFUAG_1, MBANK_A);
		pb2dfu_set_bank(DFUAG_2, MBANK_B);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*Z*/, 0);
		pb2dfu_set_addr(DFUAG_1/*A*/, pb_offset+0);
		pb2dfu_set_addr(DFUAG_2/*B*/, 10);
		// incrementy
		pb2dfu_set_inc(DFUAG_0/*Z*/, 2);
		pb2dfu_set_inc(DFUAG_1/*A*/, 2);
		pb2dfu_set_inc(DFUAG_2/*B*/, 0);
		// cnt
		pb2dfu_set_cnt(steps); // FIXME: spadne pblaze-llc: Assertion `DestReg == VirtReg && "Unknown load situation!"' failed.
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VMUL);

		/*
		pb2dfu_set_inc(DFU_MEM_A, 1);
		pb2dfu_set_inc(DFU_MEM_Z, 1);
		pb2dfu_set_cnt(2*steps-1); // HACK: should be "2*steps"
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_OP_VZ2A);
		*/
		// TODO: A[0:1:] <= Z[0:1:] (cnt=2*steps)
		// TODO: A[pb_offset+0:1:] <= Z[0:1:] (cnt=2*steps)
		// banky
		pb2dfu_set_bank(DFUAG_0, MBANK_A);
		pb2dfu_set_bank(DFUAG_1, MBANK_C);
		// offsety
		pb2dfu_set_addr(DFUAG_0/*A*/, pb_offset+0);
		pb2dfu_set_addr(DFUAG_1/*Z*/, 0);
		// incrementy
		pb2dfu_set_inc(DFUAG_0/*A*/, 1);
		pb2dfu_set_inc(DFUAG_1/*Z*/, 1);
		// cnt
		pb2dfu_set_cnt(2*steps);
		// op
		pb2dfu_wait4hw();
		pb2dfu_restart_op(DFU_VCOPY);

		/*
		pb2dfu_set_inc(DFU_MEM_A, 2);
		pb2dfu_set_inc(DFU_MEM_Z, 2);
		pb2dfu_wait4hw();
		pb2mb_eoc('.');
		*/
		pb2dfu_wait4hw();
		mbpb_exchange_data(0); // EOC
	}

	/*
	pb2mb_eoc('.');
	pb2mb_req_reset('.');
	pb2mb_reset();
	*/
	mbpb_exchange_data(0);

	while (1)
		;
}
