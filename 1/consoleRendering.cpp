#include<iostream>
#include<concrt.h>
#include<Windows.h>
#include<stdlib.h>
#include<algorithm>
using namespace std;

static constexpr int X = 40;
static constexpr int Y = 30;
static constexpr float MaxDist = 100.f;
static constexpr float SurfaceDist = 0.001f;

struct float3
{
	float x, y, z;
	float3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
	float3(float s) : x(s), y(s), z(s) {}

	float3 operator*(const float& rhs)
	{
		return float3(x * rhs, y * rhs, z * rhs);
	}

	float3 operator/(const float& rhs)
	{
		return float3(x / rhs, y / rhs, z / rhs);
	}

	float3 operator/(const float3& rhs)
	{
		return float3(x / rhs.x, y / rhs.y, z / rhs.z);
	}

	float3 operator*(const float3& rhs)
	{
		return float3(x * rhs.x, y * rhs.y, z * rhs.z);
	}

	float3 operator+(const float3& rhs)
	{
		return float3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	float3 operator-(const float3& rhs)
	{
		return float3(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	float3 operator-()
	{
		return float3(-x, -y, -z);
	}

	static float3 abs(const float3& rhs)
	{
		return float3(::abs(rhs.x), ::abs(rhs.y), ::abs(rhs.z));
	}

	static float3 Min(const float3& rhs, const float3& a)
	{
		return float3(min(rhs.x, a.x), min(rhs.y, a.y), min(rhs.z, a.z));
	}

	static float3 Max(const float3& rhs, const float3& a)
	{
		return float3(max(rhs.x, a.x), max(rhs.y, a.y), max(rhs.z, a.z));
	}

};

float length(float3 vec)
{
	return sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

float3 normalize(float3 vec)
{
	return vec / length(vec);
}

float dot(float3 a, float3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float lerp(float t1, float t2, float k)
{
	k = (1 - k) * t1 + k * t2;

	return k;
}

float clamp(float x, float _min, float _max)
{
	return x > _max ? _max : x < _min ? _min : x;
}

int clampi(int x, int _min, int _max)
{
	return x > _max ? _max : x < _min ? _min : x;
}

float smoothStep(float t1, float t2, float k)
{
	k = clamp((k - t1) / (t2 - t1), 0, 1);

	k = k * k * (3 - 2 * k);

	return k;
}

float sdfSphere(float3 _point, float3 sphere, float radius)
{
	return length(_point - sphere) - radius;
}

float sdfPlane(float3 _point, float3 plane, float3 planeNormal)
{
	return dot((_point - plane), planeNormal);
}

float sdf3dTorus(float3 _point, float3 torus, float inside, float thickness, float3 torusPlaneNormal)
{
	_point = _point - torus;
	float projPlaneFactor = dot(_point, torusPlaneNormal);
	float3 projPlane = _point - torusPlaneNormal * projPlaneFactor;
	float projPlaneToTorus = length(projPlane) - inside;

	return sqrt(projPlaneToTorus * projPlaneToTorus + projPlaneFactor * projPlaneFactor) - thickness;
}

float sdf3dBox(float3 _point, float3 box, float3 boxSize)
{
	_point = float3::abs(_point - box) - boxSize;
	return length(float3::Max(_point, float3( 0.f,0.f,0.f ))) + min( max( max(_point.x, _point.y), _point.z), 0.f);
}

float sdf3dCapsule(float3 _point, float3 A, float3 B, float radius)
{
	float3 ab = B - A;
	float3 ap = _point - A;
	float t = clamp(dot(ap, ab) / dot(ab, ab),0.f,1.f);

	return length(ap - ab * t) - radius;
}

float sdf3dCylinder(float3 _point, float3 A, float3 B, float radius)
{
	float3 ab = B - A;
	float3 pa = _point - A;

	float Len_ab = length(ab);
	float invLen_ab = 1.f / Len_ab;
	float t = dot(ab, pa) * invLen_ab;
	float3 proj_ap = A + ab * t * invLen_ab;

	float d = length(_point - proj_ap) - radius;
	float y = (abs(t * invLen_ab - 0.5f) - 0.5f) * Len_ab;

	float e = length(float3::Max(float3(d, y, 0.f), float3(0.f, 0.f, 0.f)));
	float i = min(max(d, y), 0.f);

	return e + i;
}

float sdf3dCone(float3 _point, float3 cone,float rad,  float3 direction, float height)
{
	_point = _point - cone + direction * height;

	float dh = dot(_point, direction);
	float dr = length(_point - direction* dh);
	float fp = clamp((rad * dr - height * dh) / (height * height + rad * rad), 0.f, 1.f);

	float dCircle = length(float3( max(dr - rad, 0.f), dh + height, 0.f ));

	float u = dr - fp * rad;
	float v = dh + fp * height;
	float dCone = length(float3( u, v ,0.f));

	return (-height < dh&& u < 0.f && v < 0.f) ? -min(dCone, dCircle) : min(dCone, dCircle);
}

float GetDist(float3 pos, float time)
{
	float dist;

	//점과 점 사이의 거리
	float3 targetPoint = float3(0.f,3.f,0.f);
	float rad = 3.f;

	//rad += sinf(time*5.f)*2.f;

	// float s = sinf(time * 3.f);
	// s *= s;
	// s *= s;
	// pos.y -= smoothStep(1.f,0.3f,s)*3.f -1.5f;
	// pos.y *= s*1.5f + 1.f;

	dist = length(pos - targetPoint) - rad;

	//dist = min(dist, sdf3dTorus(pos, targetPoint, rad + 2.5f, 1.f, float3(0.f,sinf(time),cosf(time))));


	return dist;
}

float RayMarching(float3 origin, float3 dir, float time)
{
	float hitDist = 0.f;
	for (int i = 0; i < 256; ++i)
	{
		float3 ray = origin + dir * hitDist;
		float currDist = GetDist(ray, time);
		if (hitDist > MaxDist || currDist < SurfaceDist)//실패 또는 성공
			break;
		hitDist += currDist;
	}

	return hitDist;
}


float3 GetNormal(float3 pos, float time)
{
	//유한 차분법

	float h = SurfaceDist;
	float diffX = GetDist(pos + float3(h, 0.f, 0.f), time) - GetDist(pos - float3(h, 0.f, 0.f), time);
	float diffY = GetDist(pos + float3(0.f, h, 0.f), time) - GetDist(pos - float3(0.f, h, 0.f), time);
	float diffZ = GetDist(pos + float3(0.f, 0.f, h) ,time) - GetDist(pos - float3(0.f, 0.f, h) ,time);

	return normalize(float3(diffX, diffY, diffZ));
}

float GetLight(float3 pos, float time, float3 normal)
{
	float3 lightPos = float3( sinf(time*3.f) * 6.f,  4.f,cosf(time*3.f) * 6.f );
	float3 lightDir = normalize(pos - lightPos);
	float3 reflectDir = -lightDir;
	float lightIntensity = max(dot(normal, reflectDir), 0.f);

	return lightIntensity;
}

void CursorView(bool show)
{
	HANDLE hConsole;
	CONSOLE_CURSOR_INFO ConsoleCursor;

	hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

	ConsoleCursor.bVisible = show;
	ConsoleCursor.dwSize = 1;

	SetConsoleCursorInfo(hConsole, &ConsoleCursor);
}

void gotoxy(short x, short y)
{
	COORD coord = { x,y };
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
}

float sphereIntersect(float3 ro, float3 rd, float3 sphere, float rad)
{
    float3 rop = ro - sphere;
    
    float b = dot(rop, rd);
    float ac = dot(rop, rop) - rad * rad;
    
    float h = b * b - ac;
    
    if (h < 0.f)//교차하지 않음
        return -1.f;
        
    h = sqrt(h);
	float t1 = -b - h;//교차점1
	float t2 = -b + h;//교차점2
    return t1 > 0.f ? t1 : t2;
}

float3 GetSphereNormal(float3 pos, float3 sphere)
{
    return normalize(pos - sphere);
}

int main()
{
	ios::sync_with_stdio(false);
	cout.tie(nullptr);
	CursorView(false);

	float3 rayOrigin = float3( 0.f,3.f,-8.f );//시작점

	float aspect = (float)X / Y;
	float time = -3.f;

	static constexpr char c[] = ".,:;-~=!*#$@";//빛의 세기

	while (true)
	{
		for (int v = 0; v < Y; ++v)
		{
			for (int u = 0; u < X; ++u)
			{
				//광선의 방향벡터
				float3 rayDir = float3((2.f * (0.5f + u) - X) / Y, -(2.f * (0.5f + v) - Y) / Y, 1.f);
				rayDir = normalize(rayDir);

			#if 1
				float marchDist = RayMarching(rayOrigin, rayDir, time);
			
				if (marchDist > MaxDist)//실패
					cout << " ";
				else//성공
				{
					float3 currPos = rayOrigin + rayDir * marchDist;
					float intensity = GetLight(currPos, time, GetNormal(currPos, time));
					int N = (int)roundf(intensity * (_countof(c) - 2));

					cout << c[clampi(N,0, (_countof(c) - 2))];
				}
			#else
				float3 spherePos = float3(0.f, 3.f, 0.f);
				float intersectDist = sphereIntersect(rayOrigin, rayDir,spherePos, 3.f);
				if(intersectDist < -0.f) //교차 실패
					cout << " ";
				else //교차 성공
				{
					float3 currPos = rayOrigin + rayDir * intersectDist;//직선에 방적식에 t1을 대입하여 얻은 교차점
					float intensity = GetLight(currPos, time, GetSphereNormal(currPos, spherePos));
					int N = (int)roundf(intensity * (_countof(c) - 2));//문자열 index

					cout << c[clampi(N, 0, (_countof(c) - 2))];
				}


			#endif
			}
			cout << "\n";//다음 줄
		}
		gotoxy(0, 0);//화면 공간의 끝에 도착하여 다시 처음으로 돌아갑니다.
		time += 0.05f;//시간의 흐름
	}

}