
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

struct lock cross_lock; //교차로에 진입하기 위한 lock

const struct position enter_cross[4] = {{2,2},{2,4},{4,2},{4,4}};
const struct position exit_cross[8] = {{1,2},{1,4},{5,2},{5,4},{2,1},{4,1},{2,5},{4,5}};

static int total_vehicle_cnt; // 전체(남은) vehicle 개수
static int finished_vehicle_cnt; // 이동을 완료한(움직였거나/가만히있거나) vehicle 개수

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
	/* from A */ {
		/* to A */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	}
};

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

//교차로에 진입할 차인지 확인하는 함수
static int is_enter_cross(struct position pos)
{
	int i;
	for(i=0;i<4;i++){
		if(pos.row == enter_cross[i].row && pos.col == enter_cross[i].col)
			return 1;
	}
	return 0;
}

//교차로에서 나간 차인지 확인하는 함수
static int is_exit_cross(struct position pos)
{
	int i;
	for(i=0;i<8;i++){
		if(pos.row == exit_cross[i].row && pos.col == exit_cross[i].col)
			return 1;
	}
	return 0;
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;

	pos_next = vehicle_path[start][dest][step];
	pos_cur = vi->position;

	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			/* actual move */
			vi->position.row = vi->position.col = -1;
			/* release previous */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			return 0;
		}
	}


	//다음에 교차로에 진입해야 하면, 일단 cross_lock부터 시도
	if (is_enter_cross(pos_next)) {
		//이미 가지고 있지 않을 때만 시도
		if(!lock_held_by_current_thread(&cross_lock)) {
			// lock_acquire(&cross_lock);
			if(!lock_try_acquire(&cross_lock)) { //교차로 입장권을 얻지 못했으면
				return -1; // -> 이동하지 못하고 턴을 끝냄(제자리에 머무름)	 
			}
		}
	} //여기까지 오면 cross_lock을 성공적으로 획득한 상태가 됨

	/* lock next position */
	// lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
	if(!lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])) { // 다음 이동할 좌표의 lock을 시도 
		return -1; // -> 이동하지 못하고 턴을 끝냄(제자리에 머무름)	 
	}
	else // lock을 성공적으로 획득했을 때(=이동할 수 있을 때)
	{
		if (vi->state == VEHICLE_STATUS_READY) {
			/* start this vehicle */
			vi->state = VEHICLE_STATUS_RUNNING;
		} else {
			/* release current position */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
		}
		/* update position */
		vi->position = pos_next;

		//cross lock을 가진 채로 교차로에서 나가면 cross lock을 해제 -> 이동한 후에 해야 안 꼬임!!?
		if(is_exit_cross(vi->position) && lock_held_by_current_thread(&cross_lock)){
			lock_release(&cross_lock);
		}
		
		return 1;
	}
}

void init_on_mainthread(int thread_cnt){
	/* Called once before spawning threads */
	lock_init(&cross_lock);
	total_vehicle_cnt = thread_cnt;
	finished_vehicle_cnt = 0;
}

void vehicle_loop(void *_vi)
{
	int result;
	int start, dest, step;

	struct vehicle_info *vi = _vi;

	start = vi->start - 'A';
	dest = vi->dest - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;

	step = 0;
	while (1) {
		/* vehicle main code */
		result = try_move(start, dest, step, vi);

		if (result == 1) { //이동을 완료했으면
			step++;
			finished_vehicle_cnt++; //
		}
		else if(result == -1) { //이동을 못했으면
			finished_vehicle_cnt++; //이동을 못했어도 움직인 것으로 간주
		}

		//모든 vehicle이 이동을 완료했으면(움직였거나 가만히 머무르면)
		if(finished_vehicle_cnt == total_vehicle_cnt){
			crossroads_step++; //unit step 증가
			finished_vehicle_cnt = 0; //이동을 완료한 vehicle 개수 초기화
		}

		/* termination condition. */ 
		if (result == 0) { //이미 종료된 vehicle이면
			total_vehicle_cnt--; //전체 vehicle 개수 감소
			break;
		}

		/* unitstep change! */
		unitstep_changed(); // 일정 시간 동안 sleep하게 됨
	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}
