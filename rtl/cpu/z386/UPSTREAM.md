# z386 upstream

Unmodified copy of the synthesizable subset of z386 by nand2mario.

- Source: https://github.com/nand2mario/z386
- Commit: b8cd03f807af34d27841386ac7c4f092e742c9a1 (2026-08-15)
- Local mirror with tests and generator scripts: `references/rtl/z386`

`*.sv` and `*.svh` are Apache-2.0 (`LICENSE`, `LICENSE-SCOPE.md`).
`ucode.hex` / `ucode.mif` hold the recovered 386 microcode and are outside
that grant. `pla_entry_rom.hex` is a generated table, unused unless
`USE_ENTRY_ROM` is defined. Upstream `biu.sv` is not copied: nothing
instantiates it.

Any local change to these files goes below this line with its date and
reason, so the copy can still be diffed against upstream.

## Local changes

- 2026-09-13 `clk_en` port on every module with clocked logic (all 56
  `always_ff`/`always @(posedge clk)` blocks gated; async-reset blocks keep
  the reset outside the enable). Applied by `.agents/tools/z386_add_ce.py`;
  the port is not called `ce` because `protection.sv` already has a PLA term
  of that name. Lets the core run one CPU clock per enable pulse under
  `clk_sys`. Upstream simple + protected regressions pass with the enable
  tied high.
- 2026-09-13 `cache_enable` port on `z386`, routed to both L1 caches (they
  already had the input, tied high).
- 2026-09-13 `l1_icache.sv`: an uncacheable instruction fetch takes the
  line-fill path without allocating instead of replicating one DWORD four
  times (the upstream bypass path corrupted branch targets, as its own
  comment warned; it was unreachable with the caches always on).
- 2026-09-13 `z386.sv`: `dbg_retire` and `dbg_next_eip` outputs (the
  instruction-completion event and `debug_ip`), so the bus unit can trace
  without hierarchical references, which Quartus 17 cannot synthesise.
- 2026-09-14 `segmentation_unit.sv`: expand-down data segments. The limit
  checker treated every segment as expand-up, so a stack segment with
  Type=6 and limit 0 (the ROM DOS on the Towns switches to one right after
  setting PE) faulted with #SS on its first push. The check now requires
  offset > limit and an access end within 0xFFFF (D/B=0) or 0xFFFFFFFF
  (D/B=1) when the cached descriptor is expand-down. Candidate for
  upstream.

- `z386.sv` reset: EDX comes up as 0x2308, the 386SX component (23H) and
  revision (08H, Am386SXL B / i386SX C step) identifier from the
  datasheets; upstream resets it to 0. The SYSTEM ROM stores DH/DL in CMOS.
- `z386.sv` dispatch: ESC opcodes D8-DF take the NOP entry (0B6) while
  CR0.EM is clear (`esc_no_fpu`). No coprocessor is fitted; upstream's
  microcode ran the register-form operand as a memory address and stored
  through it. With EM set the ESC entry still raises #NM.
- `decoder.sv`: the register selects of ESC opcodes are forced to eAX so the
  NOP entry above is XCHG eAX,eAX rather than XCHG eAX,<modrm reg>.
- 2026-09-14 CPU audit against the 386SX references and the real-mode
  SingleStepTests hardware captures (100 tests per opcode file):
  - `z386.sv`: sequential EIP is no longer truncated to 16 bits, and an
    instruction that ends past the code segment limit takes #GP(0) at its
    entry (PRM 14.7 items 7 and 8; captures at CS:IP xxxx:FFF8).
  - `segmentation_unit.sv`: real-mode SS accesses that cross offset 0 or
    FFFFh raise #SS instead of wrapping (PRM 14.7 item 7; POPAD, LEAVE,
    POPFD, ENTER, MOV SS captures).
  - `z386.sv`: the fault path no longer overwrites TMPeSP with the ESP at
    fault time, so the frame is built on the instruction-start ESP that
    the microcode restores at 89A (ENTER captures).
  - `z386.sv`: memory operand width follows `srcreg_size`, so MOVZX/MOVSX
    read a byte or word (limit check at offset FFFF); MOV, PUSH and POP of
    a segment register move a word under a 66h prefix while the 32-bit
    push/pop still steps ESP by four (0F B6 / 8C / 6607 / 6606 captures).
  - `z386.sv`: CR0 and DR7 reset to 0 (PRM table 10-1); INTR is sampled as
    a level, a request withdrawn before the instruction boundary is not
    serviced.
  - `z386.sv`: `code` output (the request is an instruction fetch) so the
    bus unit can drive D/C# low for code reads as the SX does.
- 2026-09-15 `z386.sv`: the CLZF fix-up that copies TMPC into the result
  register now applies to BSR only. BSF's bit-0 exit (uc 170) writes the
  result itself from the ZERO source; the fix-up overwrote it with whatever
  TMPC held from earlier instructions, so `BSF EAX,EAX` on a value with bit
  0 set returned stale data unless TMPC happened to be 0. RUN386's page
  allocator (BSF over a free map) mapped 64 linear pages onto one physical
  page in Bomberman. sx bench `--bsf-test`; single-step captures 0FBC /
  0FBD / 660FBC / 660FBD 2500/2500 each; upstream simple 26/26 and
  protected 34/34. Candidate for upstream.
- 2026-09-15 SX-style uncached code fetch (marty-8so.1, decision 0026):
  - `l1_icache.sv`: an uncacheable fetch goes straight from accept to the
    fill, starts at the requested word (`mem_be` skips a leading word),
    stops at the line end, and presents each arriving DWORD on
    `cpu_word_valid/idx/data`.
  - `prefetch.sv`: takes the streamed words (`pf_word_*`), skips the line
    write at the ack of a streamed fetch, sends the exact target word
    after a flush, exposes `code_abort` (the fetch in flight is dropped).
    A flush no longer sets `pf_req_toggle <= pf_ack_toggle`: the next
    cycle's `pf_can_fetch` re-toggled and the pair cancelled, so the
    redirect was refetched line-aligned from the seed branch. A request the
    paging unit has not started (`pf_taken` low) is retargeted in place.
  - `paging_unit.sv`: `pf_taken` output; the prefetch BIU wait is a flag
    (`pf_biu_wait`) instead of a state, so demand accesses proceed while an
    instruction fetch is out; `fast_pf_candidate` carries the
    `!idle_mem_precheck` term the FSM already used, closing a race where
    the icache accepted a prefetch the FSM never recorded (an orphan fill;
    with streamed words it put a line into the queue twice). Candidate for
    upstream.
  - `z386.sv`: `code_abort` output; parameter `BUS_DATA_FIRST` (default 0,
    upstream behaviour) lets the arbiter post a data read while a code
    burst is outstanding and the reverse, with code responses routed only
    while no data read is pending. The bus unit answers the data read
    first. Upstream simple 26/26 and protected 34/34 with the default.
  - `l1_cache.sv` / `z386.sv`: `store_pending` output (queued or presented
    write); a direct (I/O, INTA) request waits for it, so an OUT cannot
    pass a memory write that came before it in the program. Before, the
    code burst usually held the I/O cycle back long enough by accident;
    with the faster fetch path the memory-map ROM's E0000 bank test lost
    the second byte written before the 0404 switch. Candidate for
    upstream.
- 2026-09-15 fixes taken from nand2mario/z486 (which forked z386 at
  1deeb78) that are plain 386 behaviour, marty-g75; the z486 commit is
  named for each. Proven with the z486 test programs copied into
  `.agents/build/z386/tests` (protected suite now 45/45) plus the l1 bench:
  - `l1_cache.sv` (b65c3e4): an uncacheable read waits for the store queue
    to drain, so it cannot pass an older posted write to another address
    (Towns page-select then VRAM read). `tb_l1_cache` ordering check.
  - `l1_cache.sv` / `l1_icache.sv` (eb6f9c4): a lookup refuses a hit when a
    snoop reached that set/line on the accept edge or during the lookup
    cycle; the valid bits it sampled predate the invalidation.
  - `z386.sv` (97ab205): MOV/POP SS hold maskable interrupts off through
    the next instruction like STI. `i_pop` is now held only for an
    interrupt that will actually be dispatched (`interrupt_deliverable`);
    holding it for an inhibited one let the entry micro-op run a cycle
    before the pop, with the previous instruction's IND (`popss_shadow`).
  - `z386.sv` / `z386_pkg.sv` (fe55599): `ALUJMP_CMISC2` (3C) clears
    `misc2_flag`; the 386 gate microcode uses it at 44B/453/8CD.
  - `z386.sv` (3d47961): no bus operation leaves in the third delay uop of
    a failed protection test (`prot_redirect_prev`); VM86 port I/O always
    consults the TSS bitmap (`instr_is_port_io`). The bitmap words at
    2C7/2CC are read as words (`uc_iopb_word`; IND_DELTA holds the port's
    byte index there), and `gp_access_adj` limit-checks `RD W`/`WR W`
    micro-ops as words as z486 does. `v86_io_bitmap_iopl3`; the two VM86
    fixtures gained the I/O bitmap z486 gave them.
  - `prefetch.sv` / `paging_unit.sv` / `decoder.sv` / `z386.sv` (97ab205):
    a page fault on an instruction fetch is kept until decode starves for
    those bytes with the core idle, then raised as #PF with the fault code
    and address (before, the fetch was dropped and the core hung).
    `ifetch_page_fault`.
  - `z386.sv` / `z386_pkg.sv` (97ab205): SNOFLT/JNOFLT, SREPF/CREPF/JREP,
    STSKS/CTSKS/JSTSKL/JNTSKS, JEXTFT, JBUSY/JICEWT and the locked word bus
    ops `rd W` (05) / `wr W` (19) are decoded (`ucode_rom.sv` predecode).
    `BUSOP_IND_ALU2` names the segment for descriptor loads. LOAD_TASK's
    CS write at 76F keeps the incoming RPL as the new CPL. PREF
    suppression after a taken micro-jump applies to LOOP/Jcc only (task
    switch 7D0/7D1 needs its PREF). `task_switch_jmp386`,
    `task_switch_nested386`, `rep_stos_pf_precise`, `store_pf_precise`.
  - `z386.sv` (97ab205): TF single-step trap through 93F after each
    instruction (not after INT n / INTO taken, deferred over MOV/POP SS;
    a fault clears the sample). `tf_single_step_rm`.
  - `z386.sv` (97ab205): DIV leaves ZF set from a zero quotient (CPU
    detectors). `div_zf_386`.
  - `z386.sv` / `z386_pkg.sv` (eb6f9c4): `DEST_DR7` (19) write path;
    DR6/DR7 reset. `mov_dr_cr3_preserve`.
  - `z386.sv` (dcbefc6): #DF when a contributory or page fault hits while
    another is being delivered; a third fault raises `shutdown` (level, the
    core halts until reset). `am386sx` exports it as SHUTDOWN; the mainboard
    leaves it unconnected. `triple_fault_reset`.
  - `z386.sv`: a REP string loop that leaves through its JNOINT exit (208-
    20F puts EIP back on the REP) takes a deliverable interrupt at that
    boundary instead of restarting first; before, REP STOS ran to completion
    with the interrupt waiting. `rep_stos_intr`.
  - `z386_pkg.sv`: `phys_cacheable` (Towns memory map) was added 2026-09-14
    for marty-dar.9 and is listed here for completeness.
- 2026-09-15 audit against the extended 386 corpus (marty-5qk):
  - `z386.sv` JCNZNI (REP MOVS loop test) now also exits on a pending
    interrupt, as the die's "jump if COUNTR != 0 and no interrupt" and
    z486 do; before, a REP MOVS ran to completion with INTR/NMI waiting.
    sx bench `--rep-intr-test` (ISR at ECX=0 before, 3803 after).
    Candidate for upstream.
  - `pla_entry.svh`: real-mode/V86 IN/OUT rows (E4-E7, EC-EF, pe=0) point
    at the die entries 274/27A/29D/2A2 (LCALL PORTIO_PROTCHK) instead of
    the protected-mode JIO_OK entries. Same results and faults, but
    real-mode IN/OUT take the die's 12 clocks; 386EX capture replay dT
    went from -3/-7 to +2/-1 T-states (the EX adds 1-4 for SMM).
  - `z386.sv` DR0-DR3 and TR4-TR7 exist: register-file index 70h-77h
    (SELECT_DR_TR) stores on SBAS/SPCR and reads on LBAS/LPCR; the ModRM
    is placed in IMM for 0F 20-27 as the die does. Before, MOV DRn,r32
    wrote EAX and stored nothing. No breakpoint comparators; TR4/TR5 are
    plain storage. sx bench `dr_test` (prog-test).
  - `z386.sv` interrupt shadow also follows the ROM's RnI micro-op, so a
    real-mode MOV sreg,r inhibits INTR for one instruction like the die
    (entry 009). sx bench `sreg_shadow_test --intr-high`.
  - `z386.sv` CR0 resets to 10H: the SX datasheet and the AMD sheet give
    uuuuuu10H (ET set) in the reset table.
- 2026-09-15 `paging_unit.sv`: an access that crosses a dword boundary puts
  the part above the boundary on the bus first, as the 386 does (DX HRM
  table 3-4; every 386EX capture of a word at 4N+3 or a dword at 4N+1..3,
  memory and I/O). The TLB lookup captured with the request is the upper
  dword's when the access crosses, so PG_MEM_TLB / PG_WALKING translate and
  issue that part (`req_first`, `emit_high_half`) and PG_CROSS_* handle the
  lower one (`emit_low_half`); faults report the linear of the part that
  faulted. OPR_R merges the low part into what the upper part left
  (`opr_merge_r`). A check-only crossing now goes through PG_CROSS_PREP2 so
  its second lookup is loaded (before, PG_CROSS_TLB2 tested the first
  page's translation again). SingleStepTests data-bus order: 6689 / 668B /
  89 / 8B / E5 / ED / 6650 / 6658 whole files, 0 order mismatches.
- 2026-09-15 LOCK#: `z386.sv` marks a request `lock` when a LOCK prefix or
  XCHG with a memory operand locks the operand read, and every request up to
  and including the write that ends the read-modify-write (`lock_seq`); a
  LOCK prefix on a non-lockable instruction locks only the #UD vector read
  (captures: 18h/1Ah locked, the pushes not); `rd W` / `wr W` (descriptor
  accessed bits) and the page walker's A/D write are locked on their own.
  A request also carries `more` when another cycle of the same transfer
  follows: the lower part of a crossing access (`paging_unit.sv`) or the
  locked write after a locked read. `l1_cache.sv` carries both through the
  store queue and the bypass path; a locked read always goes to the bus
  and a locked write is never coalesced. `am386sx.sv` drives LOCK_n from
  the T1 of a locked cycle to READY# of the last one of the sequence and
  withholds HOLD until a `more` cycle's follower has run (SX datasheet
  p41/p43). sx bench `--lock-test`. Unverified against captures: LOCK# on
  the vector read of a #GP/#SS raised by a LOCK-prefixed instruction, and
  whether the walker's PTE read should be locked too (only its write is).
- 2026-09-15 TF and faults (`z386.sv`): the single-step sample taken at
  instruction start is also dropped when the ROM or the protection PLA
  raises a fault (every such routine passes through FAULT, uc 890), so a
  faulting BOUND/UD2/#NM/#GP(sel) no longer traps the handler's first
  instruction (PRM 9.4: a fault discards the pending trap). sx bench
  `tf_fault_test.asm`.
- 2026-09-15 TF with REP strings (`z386.sv`): the string loops leave for a
  pending single-step trap the way they do for an interrupt (JNOINT /
  JCNZNI test `string_loop_exit`), and the REP MOVS loop, unrolled by two
  in the ROM, takes its first-iteration JCNTZ when a trap is pending, so a
  #DB lands after every iteration (PRM 9.2). sx bench `tf_rep_test.asm`
  (REP MOVSB CX=3 -> 3 traps, REP STOSB CX=2 -> 2). Interrupts still exit
  the MOVS loop after the second copy as the ROM does.
- 2026-09-15 Double-fault classes (`z386.sv`): the fault FSM records whether
  the fault being delivered is a #PF (SCNTFF executed in the page-fault
  routine, uc 8EE/853). A #PF raised while a contributory fault's frame is
  pushed is now served first and the contributory fault re-raised on
  return; #PF on #PF and contributory on either still escalate to #DF (PRM
  Table 9-3). PM test `gp_pf_serial` (IDT entry of vector 13 in a
  not-present page: #GP -> #PF -> #GP). The #PF's CR2 lands on the second
  dword of the entry because the gate is read high dword first; unverified
  against silicon.
- 2026-09-15 IDT limit (`segmentation_unit.sv`, `z386.sv`): the interrupt
  entry read is limit-checked against IDTR in protected mode (descriptor
  reads from the IDT were exempt, and the IDT limit was never supplied to
  the checker) and routed to the ROM's #GP(SLCTR2|2) entry (uc 865), so a
  vector past the limit gives #GP(vector*8+2+EXT) with EIP on the INT
  (PRM INT n). Real mode already faulted with #GP through the 64 KB
  check. PM test `idt_limit_gp`, sx bench `idt_limit_test.asm`.
- 2026-09-15 Uncached request path (`l1_cache.sv`, `l1_icache.sv`,
  `z386.sv`): an uncached or locked data access with nothing queued ahead
  of it goes to the memory side in its accept cycle (`direct_now`), no
  lookup cycle and no store-queue pass; the bus port (`valid`, `code`,
  `addr`...) carries a cache request combinationally in the cycle it is
  presented and the `ext_*` registers only hold it until `ready`; a
  same-cycle `ready` (the upstream benches) counts as accepted at once, so
  neither the caches nor `ext_valid_r` re-issue it. Uncached code fetches
  take the same path. Measured on the sx single-step bench (caches off):
  a `push ax` write reaches T1 10 T after the first code fetch (was 16,
  386EX 9); request to T1 is 1-2 clocks (was 5-6). The request path is
  now combinational from the paging unit's live TLB through the cache
  accept and the `ext_*` mux into the bus unit's capture registers; the
  cache accept already registered that TLB cone, so the added depth is
  the two output muxes, unverified in Quartus.
- 2026-09-15 Uncached response path (`l1_cache.sv`, `l1_icache.sv`): a
  bypassed data read is handed to the paging unit in the cycle its data
  arrives (`bypass_resp_now`), and uncached code words go to the
  prefetcher the cycle they land instead of a register later. Read data
  to the consuming micro-op is 2 clocks after READY# (was 3; the bus unit
  still registers `resp_valid`, the chip uses the data the next clock).
  Flush to the target's ADS# after RET is 5 idle T (was 9).
- 2026-09-15 Crossing access second half (`paging_unit.sv`): the TLB lookup
  for the part below the dword boundary is loaded when the upper part is
  accepted (or walked), so `PG_CROSS_WAIT1` emits it the cycle the upper
  part completes and `PG_CROSS_PREP2` the cycle after a posted write;
  `PG_CROSS_TLB2` remains the walk/fault path. Two idle T between the
  halves of a misaligned word read (was 4; the chip runs them
  back-to-back, which needs a second set of OPR_R merge registers).
  ENTER with level 31 (`C8` single-step set) dT median 95 -> 50 -> 37.
- 2026-09-15 Null selector reads (`segmentation_unit.sv`): a data segment
  loaded with a null selector is marked not present in the cache and the
  limit checker faults any access through it, reads included (before, the
  FFFFFFFF base only made writes fail through the writable bit). PM test
  `null_sel_read`.
- 2026-09-15 Instruction length (`decoder.sv`, `z386.sv`): the prefix
  count holds at 15 instead of wrapping, and an instruction longer than 15
  bytes enters the ROM's OVERLONG_INST routine (uc 801, #GP(0) with EIP on
  the first prefix) instead of executing (PRM 9.9.13). sx bench
  `overlong_test.asm` (13 prefixes + MOV run, 14 fault). No 386EX capture
  exceeds 13 bytes, so the exact boundary rests on the PRM.
- 2026-09-15 PE and CS (`z386.sv`): MOV CR0 with PE 0->1 no longer clears
  CS[1:0]. `cs_rm_cached` remembers that the CS cache still holds a
  real-mode load; CPL reads 0 while it is set and the first protected-mode
  CS load (far transfer or privilege change) drops the stale RPL bits and
  clears it. PM test `pe_cs_rpl_bits` (CS=0FFBh, PE=1, MOV AX,CS reads
  0FFBh, LLDT at CPL 0, far jump to a DPL 0 segment).
- 2026-09-15 Discarded code fetch (`l1_icache.sv`, `z386.sv`, bus unit):
  `code_abort` now means the bus unit drops the fetch and answers no more
  of it, so the icache completes the uncached fill at once (`cpu_abort`),
  `icache_rd_pending` is cleared and a fetch the prefetcher has already
  given up on never reaches the bus port. Before, the bus unit answered
  the dwords still owed with one dummy response per clock and the
  prefetcher waited for the last of them. Idle T from a RET's stack read
  to the target's first ADS#: 2 (was 5; chip 2). CALL rel (`E8`) dT
  median 13 -> 7. A bus unit that keeps answering after `code_abort`
  would now corrupt the next fetch; the upstream benches run with the
  caches on, where `code_abort` never rises for cacheable code.
- 2026-09-15 Hardware breakpoints (`z386.sv`): DR0-DR3 are compared
  against DR7's L/G, R/W and LEN fields. An instruction breakpoint (R/W=00)
  at the next instruction's linear address enters the ROM's BREAKPOINT
  routine (uc 941) before the instruction unless RF is set; the queue pop
  leaves EIP on the instruction and RF is set so the pushed image resumes
  past it. A data breakpoint (01 write, 11 read/write; LEN 1/2/4 aligned)
  is collected while the instruction runs and traps after it, per
  iteration in REP loops. DR6.B0-B3 are set from the matches as the #DB
  is taken; BS still comes from the ROM. RF is cleared when an instruction
  completes except IRET and POPF, and real-mode IRETD now loads RF (PRM
  ch. 12, IRET). Not modelled: BT (TSS T bit), BD (ICE), clearing of L
  bits and LE at a task switch, RF for a task-switch JMP/CALL. sx bench
  `dr_bp_test.asm` (instruction breakpoint twice with an IRETD/RF return,
  word write breakpoint on either byte, read excluded, R/W byte read).
- 2026-09-15 LOADALL (`z386.sv`, `z386_pkg.sv`, `segmentation_unit.sv`):
  the register-file writes of the ROM's LOADALL386 routine (uc 8F6) now
  reach their targets. Slots 0-7 are the GPRs (before, every index outside
  20h-27h aliased onto a GPR, so the temporaries at the top of the image
  overwrote EDI before the descriptors were read), 8 is EIP, 9 EFLAGS,
  20h-27h the selectors ES CS SS DS FS GS LDTR TR, and 60h-69h the
  descriptor caches in the same order plus GDT/IDT, addressed through
  `resolve_seg_target` so SAR/SBAS/SLIM land on the right cache entry.
  The image's access-rights dword (G/B in bits 23:22) and its expanded
  limit are folded into the cache layout. sx bench `loadall_test.asm`
  (real mode, DS base 80000h with selector 0, EDX/EAX/EIP from the image).
  The V86 IRET and task-switch loops write the same slots and keep passing.
- 2026-09-15 Flags across a fault (`z386.sv`): the hardware checkpoint of
  EFLAGS at every instruction start is gone; FAULT restores flags only when
  the routine asked for a backup (FLGSBA sets `flags_backup_active`, the
  boundary clears it), as JNFLGB at uc 890 intends. A byte divide that
  faults on its first DIV7 step now leaves that step's flags: PF from
  AL>>1, ZF/SF/CF/AF/OF clear, matching every 386EX AAM 0 capture (D4
  2500/2500; ZF for AL<2 unverified). The AAD CF hack is confined to uc
  1A0 so a LOCK AAD #UD frame keeps CF (D5 2500/2500), and full EFLAGS
  writes leave bits 31:18 alone as the captures show across a fault.
- 2026-09-15 UMOV (`decoder.sv`): 0F 12/13 select r/m as source and reg as
  destination like MOV 8A/8B; 0F 10/11 were already right. sx bench
  `umov_test.asm` (register and memory forms, byte and word).
- 2026-09-15 TSS access faults (`z386.sv`): a hardware limit fault on an
  access made while the TSS access flag is set (STSSAF, the task-switch
  and stack-switch reads) enters the ROM's #GP/#TS(SIGMA) routine (uc 85D)
  instead of #GP(0), so JTSSAF turns it into #TS(TSS selector) as PRM
  Table 9-4 asks for a TSS limit below 67h. Other access violations keep
  #GP(0)/#SS(0), which is what the PRM specifies for them. PM test
  `tss_limit_ts` (JMP to a TSS with limit 40h -> #TS(20h)).
- 2026-09-15 NMI inside the NMI handler (`z386.sv`): an NMI edge is
  latched whether or not NMI service is in progress; only its recognition
  waits for the handler's IRET (CLRNMI), so a second NMI raised inside the
  handler runs it again afterwards (SX datasheet: NMI masked, not lost,
  until IRET). PM test `nmi_in_handler`.
- 2026-09-15 `z386.sv` BR TARGET MISMATCH diagnostic (translate_off): armed
  at i_pop and disarmed by a fault or interrupt entry, so only the branch's
  own PREF is compared. The prints during CD title boots (672 in a Bubble
  Bobble boot) were interrupt/fault handler flushes taken while the branch
  was still the latched instruction; the "actual" address was the handler.
  Same boot now prints nothing; PM test `br32_pm` (rel32 Jcc/JMP/CALL in a
  based 32-bit segment) and the 66-prefixed capture files pass.
- 2026-09-15 Reset through the ROM (`z386.sv`, `z386_pkg.sv`, `am386sx.sv`):
  after RESET the sequencer starts at ROM 000 and runs the die's reset
  routine (001 LJUMP BOOTUP, 9A6-9C3, JMP_FAR_COMMON 2F0-2F3) instead of
  waiting for the queue with `uc_active` low. What it needed: constant ROM
  0x14 (BIST signature 3DDC0C2Ch) and 0x15 (the component ID; upstream had
  0x15 misnamed `ALUSRC_PROTUN`, which no ROM word reads through the ALU),
  sources 2F/30 (BIST1/BIST2: 3DDC0C2Ch and 0 - no self-test runs, EAX
  reads 0 either way), SBAS to DESCOD (9BF, the CS base FFFF0000h; before
  only DESPTR/IRF stores landed), and JFPUOK (0x42): the reset probe reads
  as a 387 holding ERROR# low while `boot_seq` is up and `RESET_ET` is set
  (default 1: the SX comes up with ET set, datasheet Table 2.8), so 9BA/9BB
  run and CR0 ends 10h; the ESC wait loops still see no coprocessor. The
  die loads 0303h into EDX; constant 0x15 returns 2308h, the SX/Am386SXL
  component and revision. The hardware reset of EDX and CR0 is back to
  upstream's 0 (the ROM supplies both); the other reset values stay, the
  routine overwrites them. Register-file writes of the routine: loop 1
  (IRF 29h..0) zeroes the GPRs, EIP, EFLAGS and selectors (0Ah-1Fh
  temporaries are not indexed and stay at their hardware reset); loop 2
  (SBAS at 73h..71h) lands on 33h..31h because LDCNTR keeps six bits of a
  constant (POPA's 47h needs that) - a no-op either way, DR1-DR3 are
  hardware-reset; loop 3 (SAR 8200h / SLIM FFFFh at 69h..61h) plus DES_ES
  give every descriptor cache P=1 DPL=0 S=0 type 2 limit FFFFh, including
  the IDT (the PRM says 03FFh after reset; the ROM has no such constant,
  so SIDT right after reset now reports FFFFh - open question). No code is
  fetched until the routine's PREF (`boot_fetch_hold` -> `pf_suspend`) and
  its RNI is not reported on `dbg_retire`. `BOOT_UCODE` (default 1) turns
  the routine off for benches that force their own start state (the
  upstream tb_z386/tb_protected_mode/tb_test386 and tb_sx_ss pass 0);
  `am386sx` passes it through. Measured on the sx bench: the routine takes
  149 clocks (000 executes twice because the ROM pipeline holds one word
  through reset); `BOOT_WAIT_CLOCKS` = 200 - 149 is spent first so the
  fetch would leave 200 clocks after RESET falls, mid-window (datasheet
  350-450 CLK2). The first ADS# still comes 257 T after RESET, cold and
  warm, because `l1_icache.sv`/`l1_cache.sv` clear one set per clock for
  256 clocks after reset and the fetch waits for that; `--reset-test`
  prints the figure and warns outside 175-225 (not enforced until that
  walk is shortened). The reset probe now also checks EAX = 0, EDX =
  2308h and CR0 = 10h after a warm reset taken with EDX/EAX/CR0 dirty.
- 2026-09-15 PLA4 against the die table (`protection.sv`): every input
  combination `tests.txt` lists (321 rows, 16384 combinations at b13=b12=0,
  the bits the tiny PLA never sets) was run through `protection_unit` in
  `test_mode` (`.agents/tmp/audit386/compare_pla4.py`, bench
  `.agents/tmp/audit386/pla4/tb_pla4.sv`). Three tests disagreed, all in
  places where upstream had moved the decision out of the PLA:
  TST_PORTIO_BIT (04) returned #GP(0) from `descriptor_low16_nonzero`
  directly; now the tiny PLA gives p = p2 = "every tested bitmap bit
  clear" for the port test and PLA4 uses the die's terms (!p2 | !p ->
  85B). TST_SEL_ARPL (05) compared `s1_desc_rpl`/`s1_arpl_rpl` inside
  PLA4; now the tiny PLA feeds p1 = source RPL > destination RPL and p2 =
  equal, PLA4 takes !p1 -> 6B3 and !p2 -> M as the table says. TST_DES_SS
  (11) set K on the p1 & !p2 pass row and only N on PROT_TESTS_PASSED; the
  table has K|N on the pass row and nothing else (the core reads only M,
  so this changes nothing observable). After the fixes: 0 disagreements.
  Kept as is: the CPL > DPL guards in TST_DES_SIMPLE / VERR / VERW (the
  SPTR pre-validation the die does before LD_DESCRIPTOR; they only fire
  where the die's p1 would already be 1, so the table outputs match) and
  the PTOVRR conforming-code p1 formula, both tiny-PLA inferences the table
  cannot check. New PM test `arpl_rpl` (register and memory forms, both
  ZF outcomes); protected suite 53/53.
- 2026-09-15 Address-specific patches in `z386.sv` reviewed against the ROM:
  - 5D3 (JMP suppressed so LD_DESCRIPTOR falls into the accessed-bit
    write-back): removed. PRESENT_TSS is reached only from the TSS tests
    (1E/1F) and every TSS type has bit 8 set, so the condition never held.
  - 5BE gate detection at an SDEL micro-op (with its return-stack push,
    the TMPB/TMPH/TMPG/COUNTR/SIGMA set-up and the 2F3 "CS from COUNTR"
    patch) and the 5FB MORE_PRIVILEGE redirect at SET_RPL_TO_CPL: removed.
    The ROM reaches the gate code the die's way, through PLA4:
    TST_DES_JMP/CALL send a system descriptor to 5B3/5B8, PTGATE
    TST_DES_JGATE/CGATE picks CALLGATE386 (5BE) and the saved
    TST_DES_CGDEST at the second LD_DESCRIPTOR picks 5DA or 5FB. A probe
    build showed the PM suite (call_gate, call_gate_cross, interrupt
    gates, task switches) already running that path with none of the
    three patches firing; the suite passes with them gone.
  - RPTI 208-20E window: replaced by a field decode. The restart is the
    word that reloads EIP from TMPeIP with a PREF (20D); the fault paths
    (3A2, 894) reload EIP without one. `uc_restart_eip`.
  - 76F (LOAD_TASK's CS write takes the incoming RPL): kept. z386 models
    CPL as CS[1:0] and applies the K/M side effects of SET_RPL_TO_CPL,
    WRITE_RPL and COPY_STACK_DPL straight to CS[1:0]/SLCTR, so every
    other protected-mode CS write keeps those bits; the ROM carries the
    CPL inside PROTUN instead and writes CS whole. A task switch loads CS
    from the TSS with no PTGEN before 76F, so only there the full value
    is taken. Deriving this from the ROM means moving CPL into PROTUN's
    low bits and the K/M flags of PLA4, which the current CPL model does
    not support.
- 2026-09-15 ESC without a coprocessor (item kept as the NOP entry):
  constant ROM 0x1E now returns 800000F8h, the FPU opcode port (fields.txt,
  used by 3E2/4CC/4FE only), instead of SIGMA; inert while ESC takes the
  NOP entry. Evidence on what the Towns does with coprocessor cycles: the
  Technical Databook (3rd ed.) lists the 80387 as an optional card on the
  NDP slot and says CR0 records its type and presence; the Marty (386SX)
  has no such slot. The SYSTEM ROM boot path executes FNINIT (DB E3) at
  F800:4207 and F800:434F right after clearing EM and MP, so on a machine
  without an NDP the ESC cycles do complete (READY# comes back; with the
  SX's pull-ups on BUSY#/ERROR# and pull-down on PEREQ the ROM's wait
  loops fall through). What a coprocessor *read* returns (open bus) and
  whether the ASIC decodes A23 for those cycles (8000F8h against port
  00F8h) is not documented anywhere in the references; Tsugaru raises #NM
  for ESC with its FPU off (486 behaviour) and MAME's 386 core always has
  an x87, so neither is evidence. Without the read value the real ESC
  entry cannot be modelled to the observable result (FNSTSW after a
  zeroed AX would differ), so the NOP entry stays and the FPU ROM
  expectations are unchanged.
- 2026-09-15 `ucode.hex` / `ucode.mif`: words 01A and 01B are the die's
  again (`DLY` / `RNI`; upstream's `ucode_optimize.py` had folded MOV r,m
  into `RNI DLY` / `OPR_R -> DSTREG`, one micro-cycle shorter). The image
  now matches `ucode_base.hex` word for word; the mif was regenerated with
  upstream's `gen_ucode_mif.py`. 386EX capture replay dT (mean/median,
  20 tests each): 8B 2.5/3 -> 3.2/4, 8A 2.6/2 -> 3.2/3; loop_test stays
  29 T per pass (bus-bound). Upstream simple 26/26, protected 53/53.
- 2026-09-15 (evening) follow-ups after the microcode pass:
  - `l1_cache.sv` / `l1_icache.sv`: the valid-bit clear after reset runs
    beside the request FSM instead of as its first state; cacheable
    requests wait for it, uncached ones (the Marty's mode) do not, and
    `cpu_ready` says so. First ADS# after RESET 257 -> 199 T, inside the
    datasheet's 175-225 (sx `--reset-test` now enforces the window and
    IDTR limit 03FFh). The store queue drains during the walk.
  - `segmentation_unit.sv`: the reset routine's descriptor sweep leaves
    IDTR limit at 03FFh (reset table) instead of the FFFFh the loop model
    writes; `boot_seq` port. Open question noted in the plan.
  - `z386_pkg.sv` / `z386.sv`: the register-file index is 7 bits everywhere
    (`resolve_seg_target`, GPR/selector decodes, EIP/EFLAGS slots). With 6
    bits slot 27h resolved to the GDT and 60h-69h folded onto 20h-29h.
  - `pla_control.svh`: unused full ROM1 function removed; the three terms an
    earlier wider term fully covers are comments. First-match order kept:
    the captures show PUSHF/POPF push a word, which the OR of the two
    matching terms would not give. Table equivalent to upstream on all
    inputs.
  - `am386sx.sv`: an I/O write no longer empties the code stream. The SX
    keeps its queue; code that banks ROM through I/O must jump first, and
    with the look-ahead capped at 16 bytes the stale window is the chip's.
    memmap/phase-2/FPU/shutdown ROMs, sx bench 27/27, comparator PASS.
- 2026-09-15 `l1_cache.sv`: two combinational loops the fitter found in the
  direct-issue path are broken - `store_pending` no longer counts a write
  accepted in the same cycle (it is already on the bus port), and
  `direct_now` is spelt without `storeq_can_accept` (an empty queue always
  accepts). Verilator lint: no circular logic in am386sx or tb_board.
  `paging_unit.sv`: the LIVE PFN probe skips crossing requests.
- 2026-09-15 `am386sx.sv`: the pipelined data request (NA# T2P) is captured
  into `n_*` registers when it goes on the pins, so A/BHE/BLE/W_R/D_C/M_IO
  never depend on the core's live request (the fit's remaining worst path
  ran from the microcode word through the address mux to the mainboard's
  snoop and SDRAM-port captures). `Marty.sdc`: CPU register-to-register
  paths are a three-clock enable domain (decision 0040).
- 2026-09-16 `am386sx.sv`: RESET is sampled on the T-state enable (`rst_q`)
  before it reaches the bus unit and the core, so the mainboard's reset
  counter no longer fans into the bus registers on a single-clock path
  (fit 5's worst 23 paths). Recognition one T-state later; reset to first
  ADS# 200 T. Bench pin checks start one T-state into RESET.
- 2026-09-16 `l1_cache.sv` / `l1_icache.sv`: a snoop that reaches a line
  while its fill is in flight (a DMA write into data the CPU is fetching)
  leaves the fill's tag invalid instead of validating words fetched before
  the write. Before, the tag write at the end of the fill overrode the
  snoop's invalidation and the stale line served every later read (Lemmings
  level palette with the caches on; memory right, CPU wrong). tb_l1_cache
  gained the case (fails without the change).
- 2026-09-16 `am386sx.sv`: the code-stream arithmetic (fetch-ahead count,
  next fetch address, can_fetch, the next-cycle candidate) is worked out
  as if the cycle on the bus completes now; READY_n only commits it. The
  fit's READY# path ran from the SDRAM port through push/wr_hit into the
  look-ahead adders and the candidate mux. Same T-state behaviour (sx
  bench 28/28, loop 29 T, comparator 360/360); `cache_enable` is sampled
  on the enable like RESET. `rf5c68.sv`: the sample x ENV x PAN product
  is registered in the E_LOOP clock and accumulated from the register in
  E_MIX (rf5c68 bench PASS, audio bench 12/12). Second cut after the
  fit still missed by 40 ps on the RAM -> product path: sample x ENV is
  registered at the end of E_LOOP, and E_MIX multiplies that by PAN,
  applies the sign and accumulates (rf5c68 bench PASS).
- 2026-09-16 `l1_cache.sv` / `l1_icache.sv`: the valid bits and the PLRU
  state moved from flops into `ramstyle = "M10K"` arrays like the tags.
  Each array has one write statement and one read index: per way a 1-bit
  valid RAM read at `cpu_set`, one 3-bit PLRU RAM read at `cpu_set`, and
  in the icache a second valid copy read at the snoop set (registered on
  the edge that captures the snoop, so the tag-match invalidate reads
  `snoop_rd_valid*_r`). The valid port takes one writer per clock: the
  reset walk runs alone (a snoop during it is dropped, nothing is
  allocated yet), a snoop clear always writes in its own clock, and a
  fill whose validate would collide with a snoop clear on another set
  yields: the dcache holds in S_FILL one clock (`fill_done`) and writes
  tag and valid together when the port is free, the icache drops the
  validate (its victim's data and tag are untouched until then, so the
  fetch is delivered and simply not kept). A deferred-invalidate slot
  was not used because the icache sees back-to-back snoops (a DMA snoop,
  then the held CPU-write snoop) and a bounded slot still needs the fill
  to yield. The PLRU writers (reset walk, lookup hit, fill end) and the
  accept-edge read never share a clock; translate_off checks assert it.
  Expected fit effect: the 256:1 valid/PLRU read muxes and 7 x 256 flops
  with their write decoders per cache go away (about 2,400 ALUTs per
  cache in build 10) for 5 M10Ks in the dcache and 9 in the icache.
  tb_l1_cache gained three cases (snoop clear in the fill-end clock,
  snoop during the reset walk, PLRU across back-to-back hits).
- 2026-09-16 read-during-write on the target: a block RAM's mixed-port
  read of an entry written in the same clock is undefined on Cyclone V,
  where a flop array hands back the old value (found by marty-mister-9d
  in the video path). `am386sx.sv` sb_data and the rf5c68 per-channel
  register arrays are pinned to logic. The cache arrays keep M10K: their
  same-edge cases are either impossible by the FSM or have their read
  result ignored (snoop conflict guards, fill_snoop_now, PLRU assertion).
- 2026-09-16 cache coherence (marty-e0o, decision 0050):
  - `l1_icache.sv`: `fill_line` takes `fill_line_base` every clock, so a
    store snoop that lands between two fill words on an already-fetched
    word is kept; before, the merge was only committed with
    `mem_resp_valid` and the line validated stale. `tb_l1_icache_patch`.
  - `z386_pkg.sv`: `phys_rom`; `phys_cacheable(a, ram_size)` follows the
    fitted DRAM (absent expansion RAM reads FFFF and drops writes).
  - `l1_cache.sv` / `l1_icache.sv` / `z386.sv`: `ram_size` input. A write
    to a ROM image is uncacheable, its store-queue entry is `storeq_drop`
    (never forwarded to a later read) and it raises no icache patch snoop,
    since memory drops it. Board `test_rom_cache` at 2 MB and 4 MB.
  - `z386.sv`: parameter `PHYS_ADDR_BITS` (default 32). Every physical
    address handed to the caches, the bus and the snoop is folded to that
    width first. `am386sx` sets 24: the SX drives 24 pins, so a write
    through a 32-bit alias must reach the cached line (cache ROM mark 95;
    32 bits read the stale line).

- 2026-09-17 `z386_pkg.sv` / `segmentation_unit.sv`: CS is writable after
  reset and after every real-mode CS load (a 386's real-mode CS access
  byte is 93h: data, R/W, accessed; MAME i386 does the same for CPUs
  before the Pentium), and the protected-mode write check reads that bit
  instead of allowing CS writes only in V86 mode. A PM descriptor load
  still clears it. Dinosaur (Falcom 1991) sets PE without a far jump and
  writes through CS from its real-mode code segment; it faulted with #GP.
  Candidate for upstream.

- 2026-09-17 `ucode_rom.sv` / `z386.sv`: the microcode store has a write
  port (altsyncram DUAL_PORT on Quartus, same init file) and the core
  writes words 01A and 01B while in reset: the die's `DLY` / `RNI` with
  the cache option off, ROM words 084 and 01C (`RNI DLY` /
  `OPR_R -> DSTREG`, upstream's MOV r,m fold) with it on. Decision 0057.
  Capture replay: off 8A 3.2/3, 8B 3.2/4; on 8A 2.6/2, 8B 2.5/3 (mean/
  median dT, 20 tests each). The single-step bench's `+turbo` shows the
  option to the core during reset only. Local option, not for upstream.
- 2026-09-17 `z386.sv` LBAS: the DR0-3 read is taken only when the
  micro-op's destination is the register file. `COUNTR` keeps the last
  DR index after MOV DRn, so SGDT/SIDT/LBAS-based reads that followed a
  debug-register write returned DRn instead of the table base (sx bench
  `loadall_pm_test`).
- 2026-09-17 `z386.sv` savestate taps, outputs only: `dbg_halted`,
  `dbg_store_pending`, `dbg_state_sel`/`dbg_state` (the architectural
  state as 32-bit words in LOADALL table order, then CR3, CR2, DR0-3, with
  each descriptor cache folded the way LOADALL's SAR/SLIM/SBAS expect it),
  and `dbg_stopped_in_hlt`, a flag set when `single_step` ends a WIO
  stall (the HLT completed without its interrupt) and cleared by reset.
  No register input path changes.
- 2026-09-20 `z386.sv` sequencer: `instr_eip_written` (the RPTI restart
  flag a REP string instruction sets when it puts EIP back on itself for
  an interrupt) is cleared when an interrupt or fault entry starts. The
  entry microcode runs without a queue pop, so it inherited the flag and
  its RNI dropped `uc_active` at once instead of arming the delay slot;
  for a V86-to-ring-0 interrupt that slot is the `eSP <= ESP0` write, so
  the handler ran on the interrupted program's SP (EMM386 "privileged
  operation error" while TBIOSLD.SYS did a REP MOVSB). PM tests
  `v86_rep_intr_stack` (fails before, passes after) and `v86_nop_intr_stack`.
- 2026-09-20 `z386.sv` code-limit check at instruction entry uses the EIP a
  same-cycle micro-op is writing (`entry_eip`), not the register: an IRETD's
  delay slot enters the first instruction of the new code segment while the
  old EIP is still in the register, so a return from 32-bit ring 0 above
  64 KB into a 16-bit segment raised #GP(0) on its first instruction
  (Windows 3.1 VMM -> KRNL386). The three EIP write forms share
  `eip_uc_value`. PM tests `iretd_to_use16_high` (fails before), `_jcc`.
- 2026-09-20 `z386.sv` `jcc_active` is not set by a pop that enters a fault
  routine (code limit, overlong, breakpoint) and is cleared on any fault:
  the fault routine's IN=+ arithmetic took the Jcc displacement for its
  constant and read the IDT gate at IDT+68h+disp. PM tests `jcc_runoff_gate`
  (fails before), `jcc_fault_gate`, `gp_from_ldt_cs`, `gp_from_ldt_cs_paged`.
  The gate test does not check the pushed EIP: a taken Jcc whose target is
  past the limit pushes the target where a 386 pushes the Jcc (open).
