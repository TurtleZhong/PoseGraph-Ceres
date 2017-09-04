#include "new_matcher.h"


New_Matcher::New_Matcher()
{

}

New_Matcher::~New_Matcher()
{

}


int New_Matcher::SearchByProjection(Frame &frame, const std::vector<MapPoint *> &vpMapPoints)
{
    /*first we need to reproject the mappoints to current frame and */
    int nmatches = 0;

    int32_t min_id;
    double dist = 0.0;

    for(size_t iMP = 0; iMP < vpMapPoints.size(); iMP++)
    {
        double  min_cost = 10000000;
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)
        {
            continue;
        }

        const vector<size_t> vIndices = frame.getFeatureInArea(pMP->mTrackProjX,pMP->mTrackProjY, 90);

        if(vIndices.empty())
        {
            continue;
        }
        vector<int32_t> MPdescriptor = pMP->getDescriptor();
        int32_t* m1 = MPdescriptor.data();

        __m128i x1 = _mm_load_si128((__m128i*)(m1));
        __m128i x2 = _mm_load_si128((__m128i*)(m1 + 4));

        for(vector<size_t>::const_iterator vit = vIndices.begin(); vit!= vIndices.end(); vit++)
        {
            int index = *vit;
            vector<int32_t> &indicesDescriptor = frame.mvDescriptors[index];
            int32_t* m2 = indicesDescriptor.data();

            __m128i x3 = _mm_load_si128((__m128i*)(m2));
            __m128i x4 = _mm_load_si128((__m128i*)(m2+4));

            x3 = _mm_sad_epu8 (x1,x3);
            x4 = _mm_sad_epu8 (x2,x4);
            x4 = _mm_add_epi16(x3,x4); //*SAD value*//
            dist = (double)(_mm_extract_epi16(x4,0)+_mm_extract_epi16(x4,4));
            //cout << "dist = " << dist << endl;

            if(dist < min_cost)
            {
                min_cost = dist;
                min_id = index;
            }

        }

        //cout << "min cost = " << min_cost << endl;

        if(dist < 100.0)
        {
            //if(frame.mvpMapPoints[min_id] != NULL)
            //{
                frame.mvpMapPoints[min_id] = pMP;
                nmatches++;
            //}

        }

    }

    cout << BOLDGREEN"add new matches: " << nmatches << endl;
    return nmatches;

}
